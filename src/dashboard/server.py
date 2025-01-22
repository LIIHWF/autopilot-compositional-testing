from flask import Flask, Response, request, render_template, send_from_directory
from flask_cors import CORS
import subprocess
import threading
import json
import os
import pandas as pd

class PathManager:
    def __init__(self, autopilot, vista, ve):
        self.autopilot = autopilot
        self.vista = vista
        self.ve = ve
        
    def verdict_dir(self):
        save_dir = f'test_result/verdict/{self.autopilot}/{self.vista}/ve={round(self.ve, 1)}'
        return save_dir

    def verdict_path(self, test_case):
        if self.vista == 'crossing_with_traffic_lights':
            file_name = f'xf={round(test_case["xf"], 1)}.json'
        else:
            file_name = f'xf={round(test_case["xf"], 1)};xa={round(test_case["xa"], 1)}.json'
        verdict = os.path.join(self.verdict_dir(), file_name)
        return verdict
    
    def test_case_path(self):
        path = f'test_case/{self.autopilot}_{self.vista}.json'
        return path


def round_result(result):
    result = result.copy()
    for key in result:
        if isinstance(result[key], float):
            result[key] = round(result[key], 1)
    return result


ISSUE_MAP = {
    'BOTH_IN_JUNCTION': 'p1',
    'EGO_STOPPED': 'p2',
    'RUN_RED_LIGHT': 'p3',
    'IN_JUNCTION_WHEN_SIDE_GREEN': 'p4'
}

def simple_verdict(result):
    if result['verdict'] == 'collision':
        verdict = 'A'
    elif result['verdict'] == 'progress':
        verdict = 'P'
    elif result['verdict'] == 'caution':
        verdict = 'C'
    elif result['verdict'] == 'planning_failed':
        verdict = 'Fsw'
    elif result['verdict'] == 'blocking':
        verdict = 'Blk'
    else:
        raise ValueError(f'Unknown verdict: {result["verdict"]}')
    if verdict == 'A':
        if 'ego_fault' in result['safety_issues']:
            verdict += 'e'
        if 'arriving_fault' in result['safety_issues']:
            verdict += 'a'
    elif verdict in ['P', 'C']:
        if len(result['safety_issues']) == 0:
            verdict += 'S'
        else:
            verdict += 'U'
            for issue in ISSUE_MAP:
                if issue in result['safety_issues']:
                    verdict += ISSUE_MAP[issue]
    return verdict


def verdict_color(verdict):
    if not isinstance(verdict, str):
        return 'white'
    if 'Running' in verdict:
        return 'yellow'
    elif 'A' in verdict:
        return 'pink'
    elif 'PS' in verdict:
        return 'lightgreen'
    elif 'PU' in verdict:
        return 'tan'
    elif 'CS' in verdict:
        return 'lightblue'
    elif 'CU' in verdict:
        return 'plum'
    elif 'Fsw' in verdict:
        return 'orange'
    elif 'Blk' in verdict:
        return 'lightgray'
    else:
        return 'white'


class Dashboard:
    def __init__(self, autopilot: str, vista: str, ve: float):
        self.autopilot = autopilot
        self.vista = vista
        self.ve = ve
        self.path_manager = PathManager(autopilot, vista, ve)
        self.xa_index_map = dict()
        self.xf_index_map = dict()
        self.critical_test_case, self.df = self._init_table()

    def query_result(self, test_case):
        verdict_path = self.path_manager.verdict_path(test_case)
        if not os.path.exists(verdict_path):
            return None
        with open(verdict_path) as f:
            data = json.load(f)
        for key in test_case:
            if key not in data:
                return None
            if abs(test_case[key] - data[key]) > 1e-1:
                return None
        data['verdict'] = simple_verdict(data)
        return data

    def _init_table(self):
        results = []
        with open(self.path_manager.test_case_path()) as f:
            test_cases = json.load(f)
        
        critical_test_case = None
        
        for test_case in test_cases:
            if abs(test_case['ve'] - self.ve) < 1e-5:
                if critical_test_case is None:
                    critical_test_case = test_case
                result = self.query_result(test_case)
                if result is not None:
                    results.append(round_result(result))
                else:
                    result = test_case.copy()
                    result['verdict'] = 'pending'
                    results.append(round_result(result))
        
        df = pd.DataFrame(results)
        
        if self.vista != 'crossing_with_traffic_lights':
            df = df.pivot(index='xa', columns='xf', values='verdict')
        else:
            df.set_index('xf', inplace=True)
            for col in df.columns:
                if col not in ['verdict', 'xf']:
                    df.drop(col, axis=1, inplace=True)
            df = df.sort_values(by='xf')
            df = df.transpose()

        df.fillna('-', inplace=True)
        for i, xa in enumerate(df.index):
            self.xa_index_map[xa] = i + 1
        for i, xf in enumerate(df.columns):
            self.xf_index_map[xf] = i    
        return critical_test_case, df

base_dir = os.path.abspath(os.path.dirname(__file__))
app = Flask(__name__, 
    static_folder=os.path.join(base_dir, 'dist'),
    template_folder=os.path.join(base_dir, 'dist')
)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/<path:path>')
def static_file(path):
    return send_from_directory(os.path.join(base_dir, 'dist'), path)

@app.route('/autopilot_list')
def get_autopilot_list():
    autopilots = ['autoware', 'apollo', 'carla', 'lgsvl']
    return {'autopilots': autopilots}

@app.route('/scenario_type_list')
def get_scenario_type_list():
    scenario_types = ['merging', 'lane_change', 'crossing_with_yield_signs', 'crossing_with_traffic_lights']
    return {'scenario_types': scenario_types}

@app.route('/verdict_table')
def get_verdict_table():
    autopilot = request.args.get('autopilot', type=str)
    scenario = request.args.get('scenario', type=str)
    ve = request.args.get('ve', type=float)
    ve = round(ve, 1)
    dashboard = Dashboard(autopilot, scenario, ve)
    # make data be like [[...], [...], ...]
    data = dashboard.df.values.tolist()
    xas = list(dashboard.df.index)
    xfs = list(dashboard.df.columns)
    return {'data': data, 'xas': xas, 'xfs': xfs, 'critical_test_case': dashboard.critical_test_case}

@app.route('/running')
def get_running():
    if os.path.exists('test_result/running_log/running.json'):
        with open('test_result/running_log/running.json') as f:
            running = json.load(f)
            running['running'] = True
        return running
    return {'running': False}

running = False

@app.route('/start_running', methods=['POST'])
def run_on_background():
    global running
    if running:
        return {'running': True}
    print(request.json, 'in start running')
    body = request.json
    autopilot = body.get('autopilot', 'autoware').lower()
    scenario = body.get('scenario', 'merging').replace(' ', '_').lower()
    ve = body.get('ve')
    xf_lower = body.get('xfLower', 0.0)
    xf_upper = body.get('xfUpper', 320.0)
    xa_lower = body.get('xaLower', 0.0)
    xa_upper = body.get('xaUpper', 320.0)
    xf_step_max = body.get('xfMax', 40.0)
    xf_step_min = body.get('xfMin', 5.0)
    xa_step_max = body.get('xaMax', 40.0)
    xa_step_min = body.get('xaMin', 5.0)
    def run_process():
        global running
        running = True
        process = subprocess.Popen(
            f"bazel-bin/src/test_data_generator/generator {autopilot} {scenario} -ve {ve} -min_xa {xa_lower} -max_xa {xa_upper} -min_xf {xf_lower} -max_xf {xf_upper} -xa_step {xa_step_max} -xf_step {xf_step_max} -xa_min_gap {xa_step_min} -xf_min_gap {xf_step_min}",
            shell=True
        )
        process.wait()
        running = False

    running = True
    thread = threading.Thread(target=run_process)
    thread.start()
    return {'running': True}


@app.route('/single_run', methods=['POST'])
def single_run():
    global running
    print('Running status:', running)
    if running:
        return {'success': False, 'message': 'Running other test', 'result': {}}
    
    running = True
    body = request.json
    autopilot = body['autopilot'].lower()
    scenario = body['scenarioType']
    if scenario == 'Merging':
        scenario = 'merging'
    elif scenario == 'Lane change':
        scenario = 'lane_change'
    elif scenario == 'Crossing with yield signs':
        scenario = 'crossing_with_yield_signs'
    elif scenario == 'Crossing with traffic lights':
        scenario = 'crossing_with_traffic_lights'
    ve = body['ve']
    xf = body['xf']
    xa = body['xa']
    try:
        ve = float(ve)
        xf = float(xf)
        if scenario != 'crossing_with_traffic_lights':
            xa = float(xa)
        else:
            xa = None
    except:
        running = False
        return {'success': False, 'message': 'Invalid input', 'result': {}}
    scenario_save_path = f'test_result/running_log/single_scenario.json'
    os.makedirs('test_result/running_log', exist_ok=True)
    cmd = f'./script/run_single.sh {autopilot} {scenario} -ve {ve} -xf {xf} --json --scenario-save-path {scenario_save_path}'
    if scenario != 'crossing_with_traffic_lights':
        cmd += f' -xa {xa}'
    try:
        print(cmd, 'in single run')
        process = subprocess.Popen(
            cmd,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        stdout, stderr = process.communicate()
        result = stdout
    except Exception as e:
        running = False
        print(e, 'in single run')
        print(stderr, 'in single run')
        print(stdout, 'in single run')
        return {'success': False, 'message': 'Execution failed, please check the log', 'result': {}}
    # if result is not json, return error
    try:
        result = json.loads(result)
        print(result, 'in single run')
    except:
        running = False
        return {'success': False, 'message': 'Execution failed, please check the log', 'result': {}}
    running = False
    return {'success': True, 'message': 'Running test case', 'result': result}

@app.route('/plot')
def plot_png():
    autopilot = request.args.get('autopilot', type=str)
    scenario = request.args.get('scenario', type=str)
    ve = request.args.get('ve', type=float)
    xf = request.args.get('xf', type=float)
    xa = request.args.get('xa', type=float)
    has_progress = request.args.get('progress', False, type=bool)
    print(autopilot, scenario, ve, xf, xa, has_progress)
    
    ve = round(ve, 1)
    xf = round(xf, 1)
    xa = round(xa, 1)
    
    cmd = f"cat test_result/scenario/{autopilot}/{scenario}/ve={ve}/xf={xf}\\;xa={xa}.json | bazel-bin/src/visual_oracle/{autopilot}/{scenario} -t" + (" -p" if has_progress else "" )
    process = subprocess.Popen(
        cmd, 
        stdout=subprocess.PIPE,     
        stderr=subprocess.PIPE,    
        shell=True                       
    )
    stdout, stderr = process.communicate()

    if process.returncode != 0:
        return Response(f"Error: {stderr.decode('utf-8')}", status=500)
    return Response(stdout, mimetype='image/png')

@app.route('/global_view')
def global_view():
    autopilot = request.args.get('autopilot', type=str)
    scenario = request.args.get('scenario', type=str)
    ve = request.args.get('ve', type=float)
    ve = int(ve)
    # open figs/autopilot_scenario_ve.png and return
    path = f'figs/{autopilot}_{scenario}_{ve}.png'
    with open(path, 'rb') as f:
        return Response(f.read(), mimetype='image/png')


if __name__ == "__main__":
    if os.path.exists('test_result/running_log/running.json'):
        os.remove('test_result/running_log/running.json')
    CORS(app, resources={r"/*": {"origins": ["http://127.0.0.1:5173", "http://localhost:5173"]}})
    app.run(debug=True)