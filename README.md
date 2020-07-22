# sapphire-rl

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