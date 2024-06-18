
sudo pip3 install "pybind11[global]"

if [ ! -d "/usr/local/include/eigen3" ]; then
       sudo apt update
       sudo apt upgrade
       sudo apt install -y build-essential g++ git libeigen3-dev libboost-all-dev cmake
       git clone https://gitlab.com/libeigen/eigen.git
       cd eigen
       sudo mkdir /usr/local/include/eigen3
       sudo cp -r Eigen /usr/local/include/eigen3
       cd ../
       rm -rf eigen
fi
pwd
g++ -O3 -Wall -shared -std=c++17 -fPIC $(python3 -m pybind11 --includes) icp.cpp -o icp$(python3-config --extension-suffix) -I/usr/local/include/eigen3