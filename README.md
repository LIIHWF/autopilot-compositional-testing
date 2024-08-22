# Compositional Testing for Autopilots

This is the codebase for the work: [https://arxiv.org/abs/2405.16914](https://arxiv.org/abs/2405.16914)

> Rigorous Simulation-based Testing for Autonomous Driving Systems -- Targeting the Achilles' Heel of Four Open Autopilots

> Changwen Li, Joseph Sifakis, Rongjie Yan, Jian Zhang

```
@article{li2024rigorous,
  title={Rigorous Simulation-based Testing for Autonomous Driving Systems--Targeting the Achilles' Heel of Four Open Autopilots},
  author={Li, Changwen and Sifakis, Joseph and Yan, Rongjie and Zhang, Jian},
  journal={arXiv preprint arXiv:2405.16914},
  year={2024}
}
```


## Requirements

This codebase requires a Linux computer with GPU capacity needed by the Simulators. Ubuntu 22.04 operating system and an NVIDIA GPU are recommended.

## Setup Instructions

To simulate each autopilot, please clone this repository first. Our codebase has the following software dependencies:
- Bazel (installation instructions [here](https://bazel.build/install))
- Python >= 3.8 (run `bash setup_python.sh` in our codebase to install the required Python packages)

Then, install and launch the required components following the instructions below.

- For the Apollo autopilot, please install and launch: `Apollo 8.0`, `Carla 0.9.15`
- For the Autoware autopilot, please install and launch: `Autoware.Universe v1.0`, `Carla 0.9.15`
- For the Carla autopilot, please install and launch: `Carla 0.9.15`
- For the LGSVL autopilot, please install and launch: `LGSVL 2021.03 (extended by RvADS)`

### Carla 0.9.15

#### Installation

1. Download `CARLA_0.9.15.tar.gz` from [here](https://carla-releases.s3.us-east-005.backblazeb2.com/Linux/CARLA_0.9.15.tar.gz) and extract it.

#### Launch
1. From the extracted directory, run `bash CarlaUE4.sh`.


### Apollo 8.0

#### Installation

1. Install Docker following the instructions [here](https://docs.docker.com/desktop/install/linux-install/) and configure it following the instructions [here](https://docs.docker.com/engine/install/linux-postinstall/).
2. Download Apollo codebase `v8.0.0.tar.gz` from [here](https://github.com/ApolloAuto/apollo/archive/refs/tags/v8.0.0.tar.gz) and extract it into two different locations for the two moving vehicles.
3. Copy the script for the Bridge located at `patch/apollo/bridge/` in our codebase to the root of the two Apollo codebases.
4. Replace `docker/scripts/dev_start.sh` and `docker/scripts/dev_into.sh` in each Apollo codebase with the files with the same name at `patch/apollo/docker_script/` in our codebase to enable multiple autopilots on a single computer.
5. To build the first autopilot:
    1. In one of the Apollo codebases, run `bash docker/scripts/dev_start.sh v1`. Enter the created Docker container for this autopilot by running `bash docker/scripts/dev_into.sh v1`.
    3. Inside the Docker container, run `pip install loguru` and `python /apollo/bridge/patch.py`.
    3. Build the autopilot with `/apollo/apollo.sh build_opt_gpu`.
6. To build the second autopilot:
    1. In the second Apollo codebase, run `bash docker/scripts/dev_start.sh v2` to create another Docker container. Enter the created Docker container by running `bash docker/scripts/dev_into.sh v2`.
    2. Repeat the steps 5.2, 5.3, as for building the first autopilot.

#### Launch

1. (Only necessary if the Docker container has been exited.) Navigate to the two Apollo codebases, run respectively `bash docker/scripts/dev_into.sh v1` and `bash docker/scripts/dev_into.sh v2` to enter the corresponding Docker containers for the autopilots.
2. Run `bash /apollo/bridge/scripts/client.sh` in each container to enable the connection to the Bridge.

### Autoware.Universe v1.0

#### Installation

1. Install Docker following the instructions [here](https://docs.docker.com/desktop/install/linux-install/) and configure it following the instructions [here](https://docs.docker.com/engine/install/linux-postinstall/).
2. Install Rocker following the instructions [here](https://github.com/osrf/rocker).
3. Run `bash script/pull_autoware.sh` in our codebase to extract our Autoware Docker image.

#### Launch

1. Run `bash script/run_autoware.sh v1` in our codebase to start one Autoware autopilot.
2. Run `bash script/run_autoware.sh v2` in our codebase to start the other Autoware autopilot.

### LGSVL 2021.03 (extended by RvADS)

#### Installation

1. Download `lgsvl-rvads.zip` from [here](https://github.com/LIIHWF/RvADS/releases/download/v1.0/lgsvl-rvads.zip) and extract it.

#### Launch

1. Navigate to the extracted directory and run `bash run-OSSDC-SIM-v1.sh`.

## Usage

We list below the instructions for running the experiments for each autopilot.

To obtain a table of results by running a batch of test cases for given autopilot, vista type, and ego vehicle's initial speed, please execute the following instruction with arguments:

```
$ script/run_batch.sh [autopilot] [vista_tpye] -ve [number]
```
where
- `autopilot` is taken from {`apollo`, `autoware`, `carla`, `lgsvl`}
- `vista_type` is taken from {`merging`, `lane_change`, `crossing_with_yield_signs`, `crossing_with_traffic_lights`}
- `ve` is taken from 
  - {`0`, `5`, `10`, `15`} for merging and crossing with yield signs vistas; 
  - {`5`, `10`, `15`, `20`} for lane change vistas;
  - {`0`, `5`, `10`, `15`, `20`} for crossing with traffic light vistas.

Alternatively, to run a single test case, please execute the following instruction with arguments:

```
$ script/run_single.sh [autopilot] [vista_type] -ve [number] -xf [number] -xa [number]
```

where `autopilot`, `vista_type`, and `ve` are taken from the same range as in the batch mode experiment. `xf` and `xa` can be taken from 0 to 320.
