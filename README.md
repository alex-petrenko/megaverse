# voxel-rl

```
1) Clone the repo
git clone https://github.com/alex-petrenko/voxel-rl.git

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

(or install manually from https://vulkan.lunarg.com/sdk/home#linux

6) Install PyBullet
sudo apt install libbullet-dev 

7) Setup Python environment (TODO: add environment.yml to this repo, currently using one from Sample Factory)

git clone https://github.com/alex-petrenko/sample-factory.git
cd sample-factory
conda env create -f environment.yml
conda activate sample-factory

8) Update CMake if necessary (version 3.13 or newer is required)

sudo apt remove --purge cmake
hash -r
sudo snap install cmake --classic

9) Build the repo
cd voxel-rl
mkdir build
cd build

# Run CMake, specify the OpenCV build dir and Python3 executable (e.g. from a conda environment)
cmake -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR=/home/<user>/lib/opencv/build \
-DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc \
-DPYTHON_EXECUTABLE:FILEPATH=/home/<user>/miniconda3/envs/<envname>/bin/python3.7 \
../src

(optionally, add -DBULLET_ROOT=/home/<user>/lib/bullet3-2.89/install if you built it from sources)

# Build all targets
make -j10

10) Run benchmark
cd Release/bin
./voxel_env_app

(see global boolean flags in voxel_env_app.cpp, they control the scenario and rendering settings
TODO: make configurable)

11) Run viewer
cd Release/bin
./viewer

use WASD and arrows to control agent
digits (1,2) to switch between agents in multi-agent envs
E to interact with objects
O to switch to overview camera
UHJK to control overview camera

(see global vars at the top of viewer.cpp file to control which environment is chosen
TODO: make configurable)

12) To build the Python package and install it in the active Python env:
python setup.py develop
pip install -e .

13) Run tests
python -m unittest

14) You are ready to use the VoxelWorld Python API!

```

