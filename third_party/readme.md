## Livox-SDK2 + librealsense
git ls-remote https://github.com/nlohmann/json.git HEAD
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=release
make -j$(nproc)
sudo make install

sudo apt update
sudo apt install -y libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libgl1-mesa-dev
sudo apt update
sudo apt install -y libudev-dev


PCL	1.12.1	✅ 已完整安装
Eigen	3.4.0	✅ 已安装
OpenCV	4.5.4	✅ 已安装

sudo apt install ros-humble-diagnostic-updater


安装 librealsense 之后编译通过依赖 diagnostic-updater 且需要网络畅通