Previous Readme
```
1) Clone the repo
git clone https://github.com/alex-petrenko/megaverse.git

2) Init submodules
git submodule update --init --recursive

3) Clone and build OpenCV (also possible to use OpenCV installed from Conda)
cd ~/lib
git clone https://github.com/opencv/opencv.git
cd opencv
mkdir build
cmake -D CMAKE_BUILD_TYPE=Release ..
make -j10

4) Install CUDA 10.2
https://developer.nvidia.com/cuda-10.2-download-archive

5) Install Vulkan

wget -qO - https://packages.lunarg.com/lunarg-signing-key-pub.asc | sudo apt-key add -
sudo wget -qO /etc/apt/sources.list.d/lunarg-vulkan-1.2.154-bionic.list https://packages.lunarg.com/vulkan/1.2.154/lunarg-vulkan-1.2.154-bionic.list
sudo apt update
sudo apt install vulkan-sdk

(or install manually from https://vulkan.lunarg.com/sdk/home#linux, then source ./setup-env.sh to set envvars) 

6) Setup Python environment (TODO: add environment.yml to this repo, currently using one from Sample Factory)
(REQUIRES: opencv, cudatoolkit
conda install -c conda-forge opencv
conda install -c anaconda cudatoolkit
conda install -c conda-forge cudatoolkit-dev
)


git clone https://github.com/alex-petrenko/sample-factory.git
cd sample-factory
conda env create -f environment.yml
conda activate sample-factory

7) Install PyBullet
conda install -c conda-forge bullet
(preferred)

OR

sudo apt install libbullet-dev 

8) Update CMake if necessary (version 3.13 or newer is required)

conda install -c anaconda cmake
(preferred)

OR

sudo apt remove --purge cmake
hash -r
sudo snap install cmake --classic

9) Build the repo
cd megaverse
mkdir build
cd build

# Run CMake, specify the OpenCV build dir and Python3 executable (e.g. from a conda environment)
cmake -DCMAKE_BUILD_TYPE=Release  \
-DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc \
-DPYTHON_EXECUTABLE:FILEPATH=/home/<user>/miniconda3/envs/<envname>/bin/python3.7 \
../src

(optionally, add -DBULLET_ROOT=/home/<user>/lib/bullet3-2.89/install if you built it from sources)
(optionally, add -DOpenCV_DIR=/home/<user>/lib/opencv/build if you built it from sources)

# Build all targets
make -j10

10) Run benchmark
cd Release/bin
./megaverse_test_app

(see global boolean flags in megaverse_test_app.cpp, they control the scenario and rendering settings
TODO: make configurable)

11) Run viewer
cd Release/bin
./viewer_app

use WASD and arrows to control agent
digits (1,2) to switch between agents in multi-agent envs
E to interact with objects
O to switch to overview camera
UHJK to control overview camera

(see global vars at the top of viewer_app.cpp file to control which environment is chosen
TODO: make configurable)

12) To build the Python package and install it in the active Python env:
python setup.py develop
pip install -e .

13) Run tests
python -m unittest

14) You are ready to use the Megaverse Python API!

```

Training:

```

Single experiment example:

python -m megaverse_rl.train_megaverse --train_for_seconds=360000000 --train_for_env_steps=2000000000 --algo=APPO --gamma=0.997 --use_rnn=True --rnn_num_layers=2 --num_workers=12 --num_envs_per_worker=2 --num_epochs=1 --rollout=32 --recurrence=32 --batch_size=2048 --actor_worker_gpus 0 --num_policies=1 --with_pbt=False --max_grad_norm=0.0 --exploration_loss=symmetric_kl --exploration_loss_coeff=0.001 --voxel_num_simulation_threads=1 --voxel_use_vulkan=True --policy_workers_per_policy=2 --reward_clip=30 --env=voxel_env_TowerBuilding --experiment=test_cli

Example runner script:

python -m sample_factory.runner.run --run=megaverse_rl.runs.megaverse_single_agent --runner=processes --max_parallel=8 --pause_between=10 --experiments_per_gpu=2 --num_gpus=4


```


Docker setup:

```
Install docker-compose:

pip install docker-compose


```