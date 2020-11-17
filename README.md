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

11) To build the Python package and install it in the active Python env:
python setup.py develop
pip install -e .

12) TODO: run Python tests, run RL training

```


```
v4r version

1) install CUDA 10.2 (or newer?) https://developer.nvidia.com/cuda-10.2-download-archive?target_os=Linux&target_arch=x86_64&target_distro=Ubuntu&target_version=1804&target_type=runfilelocal

2) Download Vulkan SDK (tested on vulkansdk-linux-x86_64-1.2.148.1) and unzip

3) Set env vars in the current shell
cd <vulkan_sdk_folder>
source setup-env.sh

4) Update cmake if needed (tested on 3.18)
sudo snap install cmake --classic

5) Install bullet
sudo apt install libbullet-dev

6) Activate conda environment
conda activate sample-factory-pytorch-1.6

7) Cmake step
cd src
mkdir build-release
cd build-release
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_COMPILER=/usr/local/cuda-10.2/bin/nvcc ..
```
