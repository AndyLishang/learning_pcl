# 安装pcl-1.13.1

github上pcl库有[ubuntu安装指导](https://pcl.readthedocs.io/projects/tutorials/en/latest/compiling_pcl_posix.html#)。

以下为大模型给出：
```bash
# 安装依赖库
sudo apt update && sudo apt upgrade -y
sudo apt install -y git build-essential cmake cmake-gui \
libeigen3-dev libflann1.9 libflann-dev \
libboost-all-dev libqhull-dev libgtest-dev \
libvtk9.1 libvtk9-dev libvtk9-qt-dev \
libusb-1.0-0-dev libudev-dev \
libopenni-dev libopenni2-dev freeglut3-dev pkg-config

# 下载对应pcl源代码，进行编译安装
git clone -b pcl-1.13.1 https://github.com/PointCloudLibrary/pcl.git
cd pcl && mkdir build && cd build
# 编译， pcl放弃使用GPU CUDA
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_INSTALL_PREFIX=/usr/local \
         -DBUILD_apps=ON \
         -DBUILD_examples=ON \
         -DWITH_VTK=ON \
         -DWITH_QT=ON \
         -DBUILD_GPU=OFF \
         -DBUILD_CUDA=OFF
make -j4
sudo make install

# 环境变量
echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc

echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.zshrc
source ~/.zshrc
```