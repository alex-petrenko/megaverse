# voxel-rl

```
1) Clone the repo
git clone https://github.com/alex-petrenko/voxel-rl.git

2) Init submodules
git submodule update --init --recursive

3) Clone OpenCV
cd ~/lib
git clone https://github.com/opencv/opencv.git
cd opencv
mkdir build
cmake -D CMAKE_BUILD_TYPE=Release ..
make -j10

4) Build the repo
cd voxel-rl
mkdir build
cd build

# Run CMake, specify the OpenCV build dir and Python3 executable (e.g. from a conda environment)
cmake -DCMAKE_BUILD_TYPE=Release 
-DOpenCV_DIR=/home/user/all/lib/opencv/build \
-DPYTHON_EXECUTABLE:FILEPATH=/home/user/miniconda3/envs/envname/bin/python3.7 \
-DBULLET_ROOT=/home/alex/all/lib/bullet3-2.89/install \
../src

# Build all targets
make -j10

5) Run benchmark
cd Release/bin
./voxel_env_app

6) To build the Python package and install it in the active Python env:
python setup.py develop
pip install -e .

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
