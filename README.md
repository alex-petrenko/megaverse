# Megaverse

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
Note that none of the installation instructions require elevated privileges!
We bring most dependencies through conda which makes it easy to set up on a cluster without the use of Docker.

### Linux

```
1) Install VulkanSDK from https://vulkan.lunarg.com/sdk/home#linux (download and unzip), or use the following commands:
$ wget https://sdk.lunarg.com/sdk/download/1.2.162.0/linux/vulkansdk-linux-x86_64-1.2.162.0.tar.gz
$ mkdir vulkansdk && tar -xzf vulkansdk-linux-x86_64-1.2.162.0.tar.gz --directory vulkansdk

2) Add Vulkan SDK binaries to PATH (might need to do it each time recompiling Megaverse is required):
$ cd vulkansdk/1.2.162.0
$ source ./setup-env.sh

3) Clone the repo
$ git clone https://github.com/alex-petrenko/megaverse.git

4) Init submodules
$ cd megaverse 
$ git submodule update --init --recursive

5) Create a conda environment and install dependencies
$ conda create --name megaverse python=3.9
$ conda activate megaverse
$ conda install -c conda-forge opencv bullet cudatoolkit cudatoolkit-dev
$ conda install -c conda-forge 'cmake>=3.13'

(alternatively you can boostrap from an environment file: conda env create -f environment.yml)

6) Install megaverse into a conda env
$ python setup.py develop
$ pip install -e .

(Optional) 6.1) Build a .whl file to be installed elsewhere
$ python setup.py bdist_wheel
```

### macOS

Although Vulkan-powered batched rendering is not supported on macOS, a limited OpenGL version can be built for
local debugging and small-scale experiments on macOS. 
Detailed installation instruction for macOS is a welcome contribution.

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

### RL Training

Example training script using Sample Factory RL framework. First install the prerequisite:

```
pip install "sample-factory<2.0"
```

Then, to train agents in the TowerBuilding environment, execute:
```
python -m megaverse_rl.train --train_for_seconds=360000000 --train_for_env_steps=2000000000 --algo=APPO --gamma=0.997 --use_rnn=True --rnn_num_layers=2 --num_workers=10 --num_envs_per_worker=2 --ppo_epochs=1 --rollout=32 --recurrence=32 --batch_size=4096 --actor_worker_gpus 0 --num_policies=1 --with_pbt=False --max_grad_norm=0.0 --exploration_loss=symmetric_kl --exploration_loss_coeff=0.001 --megaverse_num_simulation_threads=1 --megaverse_num_envs_per_instance=30 --megaverse_num_agents_per_env=4 --megaverse_use_vulkan=True --policy_workers_per_policy=2 --reward_clip=30 --env=megaverse_TowerBuilding --experiment=TowerBuilding
```

Observe the behavior of agents by running:

```
python -m megaverse_rl.enjoy --algo=APPO --env=megaverse_TowerBuilding --experiment=TowerBuilding --megaverse_num_envs_per_instance=1 --fps=20 --megaverse_use_vulkan=True
```

### Troubleshooting

* A crash (segfault) on startup can be caused by the incorrect initialization of Vulkan device interface. Possible fixes:
    * `sudo apt remove mesa-vulkan-drivers` (unless other packages you require depend on this package)
    * Set envvar `export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json`, point to the location of `nvidia_icd.json` in your system.

### Citation

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
