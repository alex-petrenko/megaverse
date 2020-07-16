# sapphire-rl

```
1) Clone the repo
git clone https://github.com/alex-petrenko/voxel-rl.git

2) Init submodules
git submodule init --recursive

3) Clone OpenCV
cd ~/lib
git clone https://github.com/opencv/opencv.git
cd opencv
mkdir build
cmake -D CMAKE_BUILD_TYPE=Release ..
make -j10

4) Build the repo
cd voxel-rl
cmake -DOpenCV_DIR=/home/user/lib/opencv/build -D CMAKE_BUILD_TYPE=Release ..
make -j10

5) Run benchmark
cd Release/bin
./voxel_env

```