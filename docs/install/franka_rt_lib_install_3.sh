# !/bin/sh
sudo apt install build-essential cmake git libpoco-dev libeigen3-dev
git clone --recursive https://github.com/frankaemika/libfranka
cd libfranka
git checkout 0.7.1
git submodule update

mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
make
sudo make install
# improve the cpu performance
sudo apt install indicator-cpufreq
