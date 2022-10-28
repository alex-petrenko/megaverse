# Megaverse

[<img src="https://img.shields.io/discord/987232982798598164?label=discord">](https://discord.gg/4ZNdhfaZtK)

Megaverse is a dedicated high-throughput rendering and simulation engine for Artificial Intelligence research.
It features an optimized batched renderer that enables generation of up to 1,000,000 observations per second on a single machine.

* **Website:** [www.megaverse.info](https://www.megaverse.info) 
* **arXiv:** [arxiv.org/abs/2107.08170](https://arxiv.org/abs/2107.08170)

Left: RL agent completing a TowerBuilding task. Right: human player solving a randomly generated obstacle course.

<p align="middle">
<img src="https://github.com/alex-petrenko/megaverse/blob/master/data/tower_rl.gif?raw=true" width="400">
<img src="https://github.com/alex-petrenko/megaverse/blob/master/data/obstacles_hard_play_x2.gif?raw=true" width="400">
</p> 

## Installation

Currently, the easiest way to install Megaverse is to build directly from sources.

### Linux


```shell
# 0) A completely clean Linux installation needs basic OpenGL libraries. The rest of the dependencies are installed with Conda and don't require elevated privileges.
$ sudo apt install libgl1-mesa-dev libegl1-mesa-dev

# 1) Install VulkanSDK from https://vulkan.lunarg.com/sdk/home#linux (download and unzip), or use the following commands:
$ wget https://sdk.lunarg.com/sdk/download/1.2.198.1/linux/vulkansdk-linux-x86_64-1.2.198.1.tar.gz
$ mkdir vulkansdk && tar -xzf vulkansdk-linux-x86_64-1.2.198.1.tar.gz --directory vulkansdk

# 2) Add Vulkan SDK binaries to PATH (might need to do it each time recompiling Megaverse is required):
$ cd vulkansdk/1.2.198.1
$ source ./setup-env.sh

# 3) Clone the repo
$ git clone https://github.com/alex-petrenko/megaverse.git

# 4) Init submodules
$ cd megaverse 
$ git submodule update --init --recursive

# 5) Create a conda environment and install dependencies
$ conda create --name megaverse python=3.9
$ conda activate megaverse
(megaverse) $ conda install -c conda-forge 'opencv>=4.4,<4.5' 'cmake>=3.13' bullet cudatoolkit cudatoolkit-dev sdl2

# (alternatively you can boostrap from an environment file: conda env create -f environment.yml)

# 6) Install megaverse into a conda env
(megaverse) $ python setup.py develop
(megaverse) $ pip install -e .

# (Optional) 6.1) Build a .whl file to be installed elsewhere
(megaverse) $ python setup.py bdist_wheel
```

### macOS

Although Vulkan-powered batched rendering is not supported on macOS, a limited OpenGL version can be built for
local debugging and small-scale experiments on macOS.
Installing on mac is very similar to Linux, sans any Vulkan/CUDA dependencies.

```shell
# 1) Clone the repo
$ git clone https://github.com/alex-petrenko/megaverse.git

# 2) Init submodules
$ cd megaverse 
$ git submodule update --init --recursive

# 3) Create a conda environment and install dependencies
$ conda create --name megaverse python=3.9
$ conda activate megaverse
(megaverse) $ conda install -c conda-forge 'opencv>=4.4,<4.5' 'cmake>=3.13' bullet

# 4) Install megaverse into a conda env
(megaverse) $ python setup.py develop
(megaverse) $ pip install -e .

# (Optional) 4.1) Build a .whl file to be installed elsewhere
(megaverse) $ python setup.py bdist_wheel
```

### Docker 

Since installation does not require elevated priviliges, Docker setup is not required.
However, Docker-based installation is also available, see here:
[docker/README.md](https://github.com/alex-petrenko/megaverse/blob/master/docker/README.md).

## Examples

### Python API

The following script executes a random policy:

```Python
import numpy as np
from megaverse.megaverse_env import MegaverseEnv


env = MegaverseEnv(
    'TowerBuilding',
    num_envs=2, num_agents_per_env=2,
    num_simulation_threads=4, use_vulkan=True,
    params={},
)
env.reset()

while True:
    actions = [env.action_space.sample() for _ in range(env.num_agents)]
    obs, rewards, dones, infos = env.step(actions)
    if np.any(dones):
        break

    env.render()

```

## RL Training

Example training script using Sample Factory RL framework. First install the prerequisite:

```
pip install "sample-factory>=2.0"
```

(this instruction was tested on version from branch `sf2`, commit f4b5e971f467fc8dcabc0adee8b1c04885412fbb,
`pip install git+https://github.com/alex-petrenko/sample-factory.git@f4b5e971f467fc8dcabc0adee8b1c04885412fbb`)

Then, to train agents in the TowerBuilding environment, execute:
```
python -m megaverse_rl.train_megaverse --train_for_seconds=360000000 --train_for_env_steps=2000000000 --algo=APPO --gamma=0.997 --use_rnn=True --rnn_num_layers=2 --num_workers=8 --num_envs_per_worker=2 --num_epochs=1 --rollout=32 --recurrence=32 --batch_size=4096 --actor_worker_gpus 0 --num_policies=1 --with_pbt=False --max_grad_norm=0.0 --exploration_loss=symmetric_kl --exploration_loss_coeff=0.001 --megaverse_num_simulation_threads=1 --megaverse_num_envs_per_instance=48 --megaverse_num_agents_per_env=1 --megaverse_use_vulkan=True --policy_workers_per_policy=2 --reward_clip=30 --env=TowerBuilding --experiment=TowerBuilding
```

Observe the behavior of agents by running:

```
python -m megaverse_rl.enjoy --algo=APPO --env=TowerBuilding --experiment=TowerBuilding --megaverse_num_envs_per_instance=1 --fps=20 --megaverse_use_vulkan=True
```

See Sample Factory 2 documentation for additional information.

## Development

### Setting up a CMake project in IDE

The core functionality of Megaverse is implemented in C++ and uses CMake build system.
The easiest way to work on Megaverse C++ codebase is to use an IDE that can import a CMake project (defined by the root CMakeLists.txt in megaverse/src).
Any such IDE would need to run `cmake` in order to build and debug the code.
Thus `cmake` needs to be able to find all the libraries installed through conda (such as Bullet and OpenCV).

The most straightforward way to make sure that libraries can be found is to start IDE directly from the conda environment we defined above (see section Installation/Linux).
Specifically, for CLion IDE it would look like this:

```shell
$ conda activate megaverse
# navigate to Vulkan SDK installation dir
(megaverse) $ cd vulkansdk-linux-x86_64-1.2.198.1/1.2.198.1/
# make sure that Vulkan env variables are initialized
(megaverse) $ source ./setup-env.sh
# start the IDE from the terminal (assuming clion is in PATH, usually you can do this with Tools->Create Command-Line Launcher)
(megaverse) $ clion & 
```

Now in the IDE open megaverse/src/CMakeLists.txt as CMake project and you should be able to build and run targets.

#### Running an IDE without Conda enviroment

Alternatively, if IDE is not run from a conda environment we might need to explicitly specify paths to libraries in IDE's CMake command line
(i.e. in CLion that would be `Settings -> Build,Execution,Deployment -> CMake -> CMake options`).
Your CMake options might looks like this:

```
-DPYTHON_EXECUTABLE=/home/<user>/miniconda3/envs/megaverse/bin/python
-DCMAKE_CUDA_COMPILER=/home/<user>/miniconda3/envs/megaverse/bin/nvcc
-DOpenCV_DIR=/home/<user>/miniconda3/envs/megaverse/lib/cmake/opencv4
-DBUILD_GUI_APPS=ON
```

Additionally, an environment variable `VULKAN_SDK=/home/<user>/all/libs/vulkansdk-linux-x86_64-1.2.198.1/1.2.198.1/x86_64` must be set.
In most IDEs this can be set in the same CMake configuration dialogue.

Finally, CMake should be able to find Bullet physics library. There are three ways to accomplish this:
1. Install `libbullet-dev` and CMake will find the system-wide installation
2. To link against `bullet` installed by Conda you need to make sure your IDE also uses Conda's `cmake`,
rather than `cmake` bundled with the IDE. In CLion you can change this in `Settings -> Build,Execution,Deployment -> Toolchains -> CMake`.
This way `cmake` should be able to find Conda's Bullet CMake config `<env>/lib/cmake/bullet/BulletConfig.cmake`
3. Alternatively, you can build Bullet from sources and add a CMake option `-DBULLET_ROOT` pointing to the correct location.

### Notable build targets

CMakeLists.txt defines many targets. The following targets are the most useful: 

* `megaverse` builds the overall project and the Python bindings (see setup.py)
* `run_unit_tests` runs Google Tests (see `megaverse/src/test`)
* `viewer_app` builds an interactive application that allows you to control agents with keyboard and explore environments with an overview camera.
This one is really designed to interact with a single environment at a time, and is very useful during development and debugging phase. See details below.
* `megaverse_test_app` can use the parallel simulation engine and batch renderer to execute many environment at once. See details below.

### Using viewer_app

`viewer_app` can run any scenario in an interactive mode and offers a bunch of command line parameters:
```
Usage: viewer_app [options] 

Optional arguments:
-h --help           	shows help message and exits [default: false]
-v --version        	prints version information and exits [default: false]
-l --list_scenarios 	list registered scenario names [default: false]
--scenario          	name of the scenario to run [default: "ObstaclesEasy"]
--num_agents        	size of the team, pass value 1 to have just a single agent [default: 2]
--desired_fps       	rendering framerate for human perception; RL agents percieve the world at 15 FPS to avoid frameskip, hence the default value. [default: 15]
--use_opengl        	Whether to use OpenGL renderer instead of fast Vulkan renderer (currently Vulkan is only supported in Linux) [default: false]
```

Once the app started, use keyboard to control the agent and the camera:
* `WASD` and arrow keys to control the agent
* `1,2,3,4,etc.` to switch between agents (if several are present in the environment)
* Press `O` to toggle the overview camera, use mouse to control view angle
* Use `UHJK` keys to control the position of the camera in the overview mode
* Press `R` to reset the episode
* Press `ENTER` to toggle Bullet collision debug view (only OpenGL version)
* `ESC` to exit the app

### Using megaverse_test_app

`megaverse_test_app` uses parallel interface and is a much easier target to debug compared to Python Gym API. 

```
usage: megaverse_test_app [options] 

This app is designed to test the parallel execution engine and batched renderer
by simulating multiple environments at once. This app uses pretty much the same interface
as the Python Gym environment, sans the Python bindings. Whenever there is a problem
with the environment, it is much easier to debug this app directly, rather
than debugging the same code through Python.

Example, render 12 agents at the same time:
megaverse_test_app --scenario Collect --visualize --num_envs 4 --num_simulation_threads 1 --num_agents 3 --hires

Some performance figures for future reference (on 10-core Intel i9):
megaverse_test_app --scenario Empty --performance_test --num_envs 64 --num_simulation_threads 1 --num_agents 1
yields approximately 75000 FPS
megaverse_test_app --scenario Collect --performance_test --num_envs 64 --num_simulation_threads 1 --num_agents 1
yields approximately 27000 FPS


Optional arguments:
-h --help                	shows help message and exits [default: false]
-v --version             	prints version information and exits [default: false]
-l --list_scenarios      	list registered scenario names [default: false]
--scenario               	name of the scenario to run [default: "ObstaclesEasy"]
--num_agents             	size of the team [default: 2]
--use_opengl             	Whether to use OpenGL renderer instead of fast Vulkan renderer (currently Vulkan is only supported in Linux) [default: false]
--num_envs               	number of parallel environments to simulate [default: 64]
--num_simulation_threads 	number of parallel CPU threads to use for Bullet [default: 1]
--visualize              	Whether to render multiple environments on screen [default: false]
--visualize              	Whether to render multiple environments on screen [default: false]
--delay_ms               	Delay between rendered frames in milliseconds. Use only with --visualize [default: 1]
--performance_test       	Run for a limited number of env frames (currently 200000) to test performance. Uses random actions. [default: false]
--hires                  	Render at high resolution. Only use this parameter with --visualize and if the total number of agents is small [default: false]
--user_actions           	Allows the user to control agents (otherwise will use randomly generated actions). Use only with --visualize [default: false]
```

## Troubleshooting

* A crash (segfault) on startup can be caused by the incorrect initialization of Vulkan device interface. Possible fixes:
    * `sudo apt remove mesa-vulkan-drivers` (unless other packages you require depend on this package)
    * Set envvar `export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json`, point to the location of `nvidia_icd.json` in your system.

## Citation

If you use this repository in your work or otherwise wish to cite it, please make reference to our ICML2021 paper.

```
@inproceedings{petrenko2021megaverse,
  title={Megaverse: Simulating Embodied Agents at One Million Experiences per Second},
  author={Petrenko, Aleksei and Wijmans, Erik and Shacklett, Brennan and Koltun, Vladlen},
  booktitle={ICML},
  year={2021}
}
```

For questions, issues, inquiries please email apetrenko1991@gmail.com. 
Github issues and pull requests are welcome.













