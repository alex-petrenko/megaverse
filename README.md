# Megaverse

Megaverse is a new 3D simulation platform for reinforcement learning and embodied AI research.
The efficient design of the engine enables physics-based simulation with high-dimensional egocentric
observations at more than 1,000,000 actions per second on a single 8-GPU node.
Megaverse is up to 70x faster than DeepMind Lab in fully-shaded 3D scenes with interactive objects.
This high simulation performance is achieved by leveraging batched simulation,
thereby taking full advantage of the massive parallelism of modern GPUs.
Megaverse includes a new benchmark that consists of several single-agent and multi-agent tasks covering a variety of cognitive challenges.
We evaluate model-free RL on this benchmark to provide baselines and facilitate future research.

**Website:** [www.megaverse.info](https://www.megaverse.info)


### Install from prebuilt binaries

##### Linux
```
pip install git+https://github.com/alex-petrenko/megaverse/releases/download/v0.0.1/megaverse-0.0.1-cp{37m,38,39}-cp{37m,38,39}-linux_x86_64.whl
```

##### macOS

```
pip install https://github.com/alex-petrenko/megaverse/releases/download/v0.0.1/megaverse-0.0.1-cp{37,38,39}-cp{37m,38,39}-macosx_10_15_x86_64.whl
```

### Building from source
```
1) Install VulkanSDK from https://vulkan.lunarg.com/sdk/home#linux (download and unzip), or use the following commands:
$ wget https://sdk.lunarg.com/sdk/download/1.2.162.0/linux/vulkansdk-linux-x86_64-1.2.162.0.tar.gz
$ mkdir vulkansdk && tar -xzf vulkansdk-linux-x86_64-1.2.162.0.tar.gz --directory vulkansdk

2) Add Vulkan SDK binaries to PATH:
$ cd vulkansdk/1.2.162.0
$ source ./setup-env.sh

3) Clone the repo
$ git clone https://github.com/alex-petrenko/megaverse.git

4) Init submodules
$ cd megaverse 
$ git submodule update --init --recursive

5) Setup environment
$ conda env create -f environment.yml
$ conda activate megaverse

6) Install megaverse
$ python setup.py develop
$ pip install -e .
```

### Using Docker 
```shell
1) Clone the repo 
$ git clone https://github.com/alex-petrenko/megaverse.git


2) Build the image
cd megaverse
docker build -t megaverse -f docker/Dockerfile.base .

3) Start megaverse
docker run --shm-size 8G --runtime=nvidia megaverse ./docker/run.sh

(Optional) 4) Launch bash in the container and enjoy
docker run -it --shm-size 8G --runtime=nvidia --entrypoint /bin/bash megaverse
```


### RL Training

Example training script:

```shell
python -m megaverse_rl.train --train_for_seconds=360000000 --train_for_env_steps=2000000000 --algo=APPO --gamma=0.997 --use_rnn=True --rnn_num_layers=2 --num_workers=12 --num_envs_per_worker=2 --ppo_epochs=1 --rollout=32 --recurrence=32 --batch_size=2048 --actor_worker_gpus 0 --num_policies=1 --with_pbt=False --max_grad_norm=0.0 --exploration_loss=symmetric_kl --exploration_loss_coeff=0.001 --megaverse_num_simulation_threads=1 --megaverse_use_vulkan=True --policy_workers_per_policy=2 --learner_main_loop_num_cores=1 --reward_clip=30 --env=megaverse_TowerBuilding --experiment=test_cli
```

(more thorough documentation is coming)

### Troubleshooting

* A crash (segfault) on initialization can be caused by the incorrect initialization of Vulkan device interface. Possible fixes:
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
