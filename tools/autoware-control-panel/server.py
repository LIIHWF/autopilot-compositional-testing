#!/usr/bin/env python3
import argparse
import json
import os
import re
import shlex
import shutil
import signal
import subprocess
import threading
import time
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse


REPO_ROOT = Path(__file__).resolve().parents[2]
STATIC_ROOT = Path(__file__).resolve().parent / "static"
RUNTIME_ROOT = Path(__file__).resolve().parent / "runtime"
CARLA_ROOT = Path(os.environ.get("CARLA_ROOT", "/home/ray/Desktop/project/Carla"))
RECORDING_STAGE_ROOT = REPO_ROOT / "patch/autoware/autoware_data/rosbags/.active"
DEFAULT_RECORDING_ROOT = Path.home() / "AutowareRecordings"


class ProcessManager:
    def __init__(self):
        RUNTIME_ROOT.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.processes = {}
        self.log_files = {}
        self.recordings = {"v1": None, "v2": None}
        self.stack_state = {"state": "idle", "detail": "All components are stopped"}
        self.stack_cancel = threading.Event()
        self.test_recording_instance = None
        self.test_report_seen = False

    def _run(self, name, command, cwd=REPO_ROOT, env=None):
        with self.lock:
            current = self.processes.get(name)
            if current and current.poll() is None:
                return False, f"{name} is already running"
            previous_log = self.log_files.get(name)
            if previous_log and not previous_log.closed:
                previous_log.close()
            log_path = RUNTIME_ROOT / f"{name}.log"
            log_handle = open(log_path, "w" if name == "cctest" else "a", buffering=1)
            log_handle.write(f"\n[{time.strftime('%Y-%m-%d %H:%M:%S')}] $ {' '.join(command)}\n")
            process = subprocess.Popen(
                command,
                cwd=str(cwd),
                env=env or os.environ.copy(),
                stdout=log_handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                text=True,
            )
            self.processes[name] = process
            self.log_files[name] = log_handle
            return True, f"{name} started"

    def _tracked_running(self, name):
        process = self.processes.get(name)
        return bool(process and process.poll() is None)

    @staticmethod
    def _docker_running(container):
        result = subprocess.run(
            ["docker", "inspect", "--format", "{{.State.Running}}", container],
            capture_output=True,
            text=True,
        )
        return result.returncode == 0 and result.stdout.strip() == "true"

    @staticmethod
    def _carla_pids():
        pids = []
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit():
                continue
            try:
                cmdline = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="ignore")
            except (OSError, PermissionError):
                continue
            if "CarlaUE4-Linux-Shipping" in cmdline:
                pids.append(int(entry.name))
        return pids

    @staticmethod
    def _port_listening(port):
        result = subprocess.run(["ss", "-ltn"], capture_output=True, text=True)
        return any(line.split()[3].endswith(f":{port}") for line in result.stdout.splitlines()[1:] if len(line.split()) >= 4)

    def status(self):
        with self.lock:
            test = self.processes.get("cctest")
            test_state = "running" if test and test.poll() is None else "stopped"
            if test and test.poll() is not None:
                test_state = "success" if test.returncode == 0 or self.test_report_seen else "failed"
            v1_running = self._docker_running("autoware_docker_v1")
            v2_running = self._docker_running("autoware_docker_v2")
            v1_recording = self._recording_status("v1")
            v2_recording = self._recording_status("v2")
            stack = dict(self.stack_state)
            if stack["state"] not in {"starting", "stopping", "failed"}:
                running_count = sum((bool(self._carla_pids()), v1_running, v2_running))
                if running_count == 3:
                    stack = {"state": "ready", "detail": "CARLA, v1 and v2 are running"}
                elif running_count:
                    stack = {"state": "partial", "detail": f"{running_count}/3 components are running"}
                else:
                    stack = {"state": "idle", "detail": "All components are stopped"}
            return {
                "carla": {"state": "running" if self._carla_pids() else "stopped", "detail": "port 2000" if self._port_listening(2000) else "not listening"},
                "v1": {"state": "running" if v1_running else ("starting" if self._tracked_running("v1") else "stopped"), "detail": "ROS domain 1"},
                "v2": {"state": "running" if v2_running else ("starting" if self._tracked_running("v2") else "stopped"), "detail": "ROS domain 2"},
                "cctest": {"state": test_state, "detail": "port 12346 active" if self._port_listening(12346) else "idle"},
                "recordings": {"v1": v1_recording, "v2": v2_recording},
                "stack": stack,
            }

    def _recording_status(self, instance):
        recording = self.recordings.get(instance)
        if not recording:
            return {"state": "idle"}
        stage_path = recording["stage_path"]
        try:
            size = sum(item.stat().st_size for item in stage_path.rglob("*") if item.is_file()) if stage_path.exists() else 0
        except OSError:
            size = 0
        return {
            "state": "recording" if recording["process"].poll() is None else "ready",
            "name": recording["name"],
            "output_dir": str(recording["output_dir"]),
            "size_bytes": size,
            "started_at": recording["started_at"],
        }

    def shutdown(self):
        for instance in ("v1", "v2"):
            recording = self.recordings.get(instance)
            if recording:
                self.stop_recording(instance)

    def topics(self, instance):
        if instance not in {"v1", "v2"}:
            return False, "Invalid Autoware instance"
        if not self._docker_running(f"autoware_docker_{instance}"):
            return False, f"Autoware {instance} is not running"
        try:
            result = subprocess.run(
                ["docker", "exec", f"autoware_docker_{instance}", "bash", "-lc", "source /autoware/install/setup.bash && ros2 topic list -t"],
                capture_output=True,
                text=True,
                timeout=20,
            )
        except subprocess.TimeoutExpired:
            return False, "Timed out while discovering topics"
        if result.returncode != 0:
            return False, result.stderr.strip() or "Unable to list topics"
        topics = []
        for line in result.stdout.splitlines():
            match = re.match(r"^(\S+)\s+\[(.+)]$", line.strip())
            if match:
                topics.append({"name": match.group(1), "types": match.group(2)})
        return True, topics

    @staticmethod
    def _safe_output_dir(raw_path):
        path = Path(raw_path or DEFAULT_RECORDING_ROOT).expanduser().resolve()
        home = Path.home().resolve()
        if path != home and home not in path.parents:
            raise ValueError(f"Recording directory must be inside {home}")
        path.mkdir(parents=True, exist_ok=True)
        if not os.access(path, os.W_OK):
            raise ValueError("Recording directory is not writable")
        return path

    def start_recording(self, payload):
        instance = payload.get("instance", "v1")
        if instance not in {"v1", "v2"}:
            return False, "Invalid Autoware instance"
        if not self._docker_running(f"autoware_docker_{instance}"):
            return False, f"Autoware {instance} is not running"
        with self.lock:
            current = self.recordings.get(instance)
            if current and current["process"].poll() is None:
                return False, f"Autoware {instance} is already being recorded"
            try:
                output_dir = self._safe_output_dir(payload.get("output_dir"))
            except (OSError, ValueError) as error:
                return False, str(error)
            default_name = f"autoware_{instance}_{time.strftime('%Y%m%d_%H%M%S')}"
            name = (payload.get("name") or default_name).strip()
            if not re.fullmatch(r"[A-Za-z0-9._-]{1,96}", name):
                return False, "Recording name may contain only letters, numbers, dots, underscores and hyphens"
            final_path = output_dir / name
            if final_path.exists():
                return False, f"Recording already exists: {final_path}"
            all_topics = bool(payload.get("all_topics", True))
            topics = payload.get("topics") or []
            if not all_topics:
                valid_topics = all(isinstance(topic, str) and re.fullmatch(r"/[A-Za-z0-9_~/{}.-]+", topic) for topic in topics)
                if not topics or not valid_topics:
                    return False, "Select at least one valid topic"
                topics = list(dict.fromkeys(topics))
            token = f"{instance}-{int(time.time())}-{os.getpid()}"
            stage_path = RECORDING_STAGE_ROOT / token
            RECORDING_STAGE_ROOT.mkdir(parents=True, exist_ok=True)
            home_result = subprocess.run(
                ["docker", "exec", f"autoware_docker_{instance}", "bash", "-lc", "printf %s \"$HOME\""],
                capture_output=True,
                text=True,
                timeout=8,
            )
            container_home = home_result.stdout.strip()
            if home_result.returncode != 0 or not container_home.startswith("/"):
                return False, "Unable to determine the Autoware container home directory"
            container_path = f"{container_home}/autoware_data/rosbags/.active/{token}"
            record_args = ["ros2", "bag", "record"] + (["-a"] if all_topics else topics) + ["-o", container_path]
            command = "source /autoware/install/setup.bash && exec " + " ".join(shlex.quote(arg) for arg in record_args)
            log_path = RUNTIME_ROOT / f"recording-{instance}.log"
            log_handle = open(log_path, "a", buffering=1)
            process = subprocess.Popen(
                ["docker", "exec", f"autoware_docker_{instance}", "bash", "-lc", command],
                cwd=str(REPO_ROOT), stdout=log_handle, stderr=subprocess.STDOUT,
                start_new_session=True, text=True,
            )
            self.recordings[instance] = {
                "process": process, "log_handle": log_handle, "stage_path": stage_path,
                "container_path": container_path, "output_dir": output_dir,
                "final_path": final_path, "name": name, "started_at": int(time.time()),
            }
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline and not stage_path.exists() and process.poll() is None:
                time.sleep(0.1)
            if process.poll() is not None or not stage_path.exists():
                log_handle.close()
                self.recordings[instance] = None
                return False, f"Recorder for Autoware {instance} did not become ready"
            return True, f"Recording Autoware {instance}"

    def stop_recording(self, instance):
        if instance not in {"v1", "v2"}:
            return False, "Invalid Autoware instance"
        with self.lock:
            recording = self.recordings.get(instance)
            if not recording:
                return False, f"Autoware {instance} is not being recorded"
            process = recording["process"]
            if process.poll() is None:
                pattern = "[r]os2 bag record.*" + recording["container_path"]
                subprocess.run(
                    ["docker", "exec", f"autoware_docker_{instance}", "bash", "-lc", "pkill -INT -f " + shlex.quote(pattern)],
                    capture_output=True, text=True, timeout=8,
                )
                try:
                    process.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=8)
            recording["log_handle"].close()
            stage_path = recording["stage_path"]
            if not stage_path.exists():
                self.recordings[instance] = None
                return False, "Recorder stopped but no bag data was produced"
            try:
                shutil.move(str(stage_path), str(recording["final_path"]))
            except OSError as error:
                return False, f"Bag is safe in {stage_path}, but moving it failed: {error}"
            final_path = recording["final_path"]
            self.recordings[instance] = None
            return True, f"Saved recording to {final_path}"

    def start_component(self, name):
        if name == "carla":
            if self._carla_pids():
                return False, "CARLA is already running"
            launcher = CARLA_ROOT / "CarlaUE4.sh"
            if not launcher.exists():
                return False, f"CARLA launcher not found: {launcher}"
            return self._run("carla", ["bash", str(launcher)], cwd=CARLA_ROOT)
        if name in {"v1", "v2"}:
            if self._docker_running(f"autoware_docker_{name}"):
                return False, f"Autoware {name} is already running"
            other = "v2" if name == "v1" else "v1"
            if self._tracked_running(other) and not self._docker_running(f"autoware_docker_{other}"):
                return False, f"Autoware {other} is still starting; wait until it is ready"
            return self._run(name, ["bash", "script/run_autoware.sh", name])
        return False, "Unknown component"

    def _wait_until(self, predicate, timeout, message, cancellable=True):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if cancellable and self.stack_cancel.is_set():
                raise InterruptedError("Environment startup was cancelled")
            if predicate():
                return
            time.sleep(0.5)
        raise TimeoutError(message)

    def _set_stack_state(self, state, detail):
        with self.lock:
            self.stack_state = {"state": state, "detail": detail}

    def _ensure_stack_active(self):
        if self.stack_cancel.is_set():
            raise InterruptedError("Environment startup was cancelled")

    def start_stack(self):
        with self.lock:
            if self.stack_state["state"] in {"starting", "stopping"}:
                return False, "An environment operation is already in progress"
            self.stack_cancel.clear()
            self.stack_state = {"state": "starting", "detail": "Starting CARLA"}
        threading.Thread(target=self._start_stack_worker, daemon=True).start()
        return True, "Environment startup started"

    def _start_stack_worker(self):
        try:
            if not self._carla_pids():
                ok, message = self.start_component("carla")
                if not ok:
                    raise RuntimeError(message)
            self._wait_until(lambda: self._port_listening(2000), 120, "CARLA did not open port 2000 in time")
            self._ensure_stack_active()
            self._set_stack_state("starting", "Starting Autoware v1")
            if not self._docker_running("autoware_docker_v1"):
                ok, message = self.start_component("v1")
                if not ok:
                    raise RuntimeError(message)
            self._wait_until(lambda: self._docker_running("autoware_docker_v1"), 180, "Autoware v1 did not start in time")
            self._ensure_stack_active()
            self._set_stack_state("starting", "Starting Autoware v2")
            if not self._docker_running("autoware_docker_v2"):
                ok, message = self.start_component("v2")
                if not ok:
                    raise RuntimeError(message)
            self._wait_until(lambda: self._docker_running("autoware_docker_v2"), 180, "Autoware v2 did not start in time")
            self._ensure_stack_active()
            self._set_stack_state("ready", "CARLA, v1 and v2 are running")
        except InterruptedError:
            return
        except Exception as error:
            self._set_stack_state("failed", str(error))

    def stop_stack(self):
        with self.lock:
            if self.stack_state["state"] == "stopping":
                return False, "An environment operation is already in progress"
            self.stack_cancel.set()
            self.stack_state = {"state": "stopping", "detail": "Stopping tests and recordings"}
        threading.Thread(target=self._stop_stack_worker, daemon=True).start()
        return True, "Environment shutdown started"

    def _stop_stack_worker(self):
        try:
            self.stop_test()
            for instance in ("v2", "v1"):
                self._set_stack_state("stopping", f"Stopping Autoware {instance}")
                if self.recordings.get(instance):
                    self.stop_recording(instance)
                self.stop_component(instance)
                self._wait_until(lambda instance=instance: not self._docker_running(f"autoware_docker_{instance}"), 30, f"Autoware {instance} did not stop in time", cancellable=False)
            self._set_stack_state("stopping", "Stopping CARLA")
            self.stop_component("carla")
            self._wait_until(lambda: not self._carla_pids(), 30, "CARLA did not stop in time", cancellable=False)
            self._set_stack_state("idle", "All components are stopped")
        except Exception as error:
            self._set_stack_state("failed", str(error))

    def stop_component(self, name):
        if name in {"v1", "v2"}:
            container = f"autoware_docker_{name}"
            subprocess.run(["docker", "stop", "--time", "10", container], capture_output=True, text=True)
            self._terminate_tracked(name)
            return True, f"Autoware {name} stopped"
        if name == "carla":
            pids = self._carla_pids()
            for pid in pids:
                try:
                    os.kill(pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
            self._terminate_tracked("carla")
            return True, "CARLA stopped"
        return False, "Unknown component"

    def _terminate_tracked(self, name):
        with self.lock:
            process = self.processes.get(name)
            if process and process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=12)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass

    def start_test(self, payload):
        autopilot = payload.get("autopilot", "autoware")
        vista = payload.get("vista", "merging")
        if autopilot != "autoware":
            return False, "This dashboard currently supports Autoware tests only"
        if self._tracked_running("cctest"):
            return False, "CCTest is already running"
        allowed_vistas = {"merging", "lane_change", "crossing_with_yield_signs", "crossing_with_traffic_lights"}
        if vista not in allowed_vistas:
            return False, "Invalid vista"
        try:
            ve = float(payload["ve"])
            xf = float(payload["xf"])
            xa_raw = payload.get("xa")
            xa = None if xa_raw in (None, "") else float(xa_raw)
        except (KeyError, TypeError, ValueError):
            return False, "ve, xf and optional xa must be numbers"
        command = ["script/run_single.sh", autopilot, vista, "-ve", str(ve), "-xf", str(xf)]
        if vista != "crossing_with_traffic_lights":
            if xa is None:
                return False, "xa is required for this vista"
            command.extend(["-xa", str(xa)])
        if payload.get("verbose", True):
            command.append("--log")
        recording = payload.get("recording")
        recording_instance = None
        if recording and recording.get("enabled"):
            ok, message = self.start_recording(recording)
            if not ok:
                return False, f"Test was not started: {message}"
            recording_instance = recording.get("instance", "v1")
        ok, message = self._run("cctest", command)
        if not ok:
            if recording_instance:
                self.stop_recording(recording_instance)
            return False, message
        self.test_report_seen = False
        self.test_recording_instance = recording_instance
        threading.Thread(target=self._monitor_test, args=(recording_instance,), daemon=True).start()
        return True, message

    def _monitor_test(self, instance):
        process = self.processes.get("cctest")
        log_path = RUNTIME_ROOT / "cctest.log"
        report_seen = False
        while process and process.poll() is None:
            try:
                if "================== Report ==================" in log_path.read_text(errors="replace"):
                    report_seen = True
                    break
            except OSError:
                pass
            time.sleep(0.25)
        if report_seen:
            self.test_report_seen = True
            time.sleep(0.25)
            self._terminate_tracked("cctest")
        if self.test_recording_instance == instance and self.recordings.get(instance):
            self.stop_recording(instance)
        self.test_recording_instance = None

    def stop_test(self):
        self._terminate_tracked("cctest")
        instance = self.test_recording_instance
        if instance and self.recordings.get(instance):
            self.stop_recording(instance)
        self.test_recording_instance = None
        return True, "CCTest stopped"

    def logs(self, target, lines=160):
        if target not in {"carla", "v1", "v2", "cctest"}:
            return "Unknown log target"
        path = RUNTIME_ROOT / f"{target}.log"
        if not path.exists():
            if target in {"v1", "v2"}:
                result = subprocess.run(["docker", "logs", "--tail", str(lines), f"autoware_docker_{target}"], capture_output=True, text=True)
                return (result.stdout + result.stderr).strip() or "No logs yet"
            return "No logs yet"
        content = path.read_text(errors="replace").splitlines()
        return "\n".join(content[-lines:])


MANAGER = ProcessManager()


class DashboardHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(STATIC_ROOT), **kwargs)

    def log_message(self, fmt, *args):
        pass

    def _json(self, payload, status=HTTPStatus.OK):
        data = json.dumps(payload).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _body(self):
        length = int(self.headers.get("Content-Length", "0"))
        if length > 65536:
            raise ValueError("Request too large")
        return json.loads(self.rfile.read(length) or b"{}")

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == "/api/status":
            self._json(MANAGER.status())
            return
        if parsed.path == "/api/logs":
            query = parse_qs(parsed.query)
            target = query.get("target", ["cctest"])[0]
            self._json({"target": target, "content": MANAGER.logs(target)})
            return
        if parsed.path == "/api/topics":
            instance = parse_qs(parsed.query).get("instance", ["v1"])[0]
            ok, result = MANAGER.topics(instance)
            self._json({"ok": ok, "topics": result if ok else [], "message": "" if ok else result}, HTTPStatus.OK if ok else HTTPStatus.CONFLICT)
            return
        super().do_GET()

    def do_POST(self):
        try:
            parts = [part for part in urlparse(self.path).path.split("/") if part]
            if parts[:2] == ["api", "components"] and len(parts) == 4:
                _, _, name, action = parts
                if action == "start":
                    ok, message = MANAGER.start_component(name)
                elif action == "stop":
                    ok, message = MANAGER.stop_component(name)
                else:
                    raise ValueError("Unknown action")
                self._json({"ok": ok, "message": message}, HTTPStatus.OK if ok else HTTPStatus.CONFLICT)
                return
            if parts == ["api", "stack", "start"]:
                ok, message = MANAGER.start_stack()
                self._json({"ok": ok, "message": message}, HTTPStatus.OK if ok else HTTPStatus.CONFLICT)
                return
            if parts == ["api", "stack", "stop"]:
                ok, message = MANAGER.stop_stack()
                self._json({"ok": ok, "message": message}, HTTPStatus.OK if ok else HTTPStatus.CONFLICT)
                return
            if parts == ["api", "test", "start"]:
                ok, message = MANAGER.start_test(self._body())
                self._json({"ok": ok, "message": message}, HTTPStatus.OK if ok else HTTPStatus.CONFLICT)
                return
            if parts == ["api", "test", "stop"]:
                ok, message = MANAGER.stop_test()
                self._json({"ok": ok, "message": message})
                return
            if parts == ["api", "recordings", "start"]:
                ok, message = MANAGER.start_recording(self._body())
                self._json({"ok": ok, "message": message}, HTTPStatus.OK if ok else HTTPStatus.CONFLICT)
                return
            if parts == ["api", "recordings", "stop"]:
                ok, message = MANAGER.stop_recording(self._body().get("instance", ""))
                self._json({"ok": ok, "message": message}, HTTPStatus.OK if ok else HTTPStatus.CONFLICT)
                return
            self._json({"ok": False, "message": "Not found"}, HTTPStatus.NOT_FOUND)
        except (ValueError, json.JSONDecodeError) as error:
            self._json({"ok": False, "message": str(error)}, HTTPStatus.BAD_REQUEST)


def main():
    parser = argparse.ArgumentParser(description="Local Autoware experiment control panel")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8787)
    args = parser.parse_args()
    server = ThreadingHTTPServer((args.host, args.port), DashboardHandler)
    print(f"CCTest Control Panel: http://{args.host}:{args.port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        MANAGER.shutdown()
        server.server_close()


if __name__ == "__main__":
    main()
