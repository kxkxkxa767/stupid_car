# 树莓派配置 opencv c++ 环境

## C/C++编译环境配置

安装C/C++编译器GCC、G++

```cpp
sudo apt install gcc
sudo apt install g++
```

安装build-essential

```cpp
sudo apt install build-essential
```

安装cmake编译工具

```cpp
sudo apt install cmake
```

## 安装相关依赖库

安装libgtk，GTK(GIMP Toolkit)是一个Linux平台下基于Xwindow图形窗口的图形用户编程接口工具,可以借助它来开发Linux平台下基于Xwindow的图形用户界面。

```cpp
sudo apt install libgtk2.0-dev
```

安装pkg-config

```cpp
sudo apt install pkg-config
```

安装ffmpeg,ffmpeg(命令行工具) 是一个快速的音视频转换工具。

```cpp
sudo apt install ffmpeg
```

安装libavcodec-dev

```cpp
sudo apt install libavcodec-dev 
```

安装libavformat-dev

```cpp
sudo apt install libavformat-dev
```

安装libswscale-dev

```cpp
sudo apt install libswscale-dev
```

## 安装OpenCV

为了方便后期管理，我们把文件下载安装到Downloads文件夹。【进入Downloads】

```cpp
cd /home/pi/Downloads
```

然后利用wget命令从网络下载opencv4.2.0

```cpp
wget -O opencv-4.2.0.zip https://github.com/opencv/opencv/archive/4.2.0.zip 
```

解压文件

```cpp
unzip opencv-4.2.0.zip
```

在opencv-4.2.0文件夹下创建build文件夹以存放编译临时文件

```cpp
mkdir build
cd /home/pi/Downloads/opencv-4.2.0/build/
```

进行cmake-make编译，然后使用cmake命令进行配置

```cpp
sudo cmake -D WITH_TBB=ON -D WITH_EIGEN=ON -D OPENCV_GENERATE_PKGCONFIG=ON  -D BUILD_DOCS=ON -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_EXAMPLES=OFF  -D WITH_OPENCL=OFF -D WITH_CUDA=OFF -D BUILD_opencv_gpu=OFF -D BUILD_opencv_gpuarithm=OFF -D BUILD_opencv_gpubgsegm=O -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ..
```

这条命令会在build目录里生成对应配置的Makefile文件，可以看到配置信息之间是通过空格和-D来分割和标示的，配置了很多信息。

直接进行编译安装

```cpp
sudo make -j8
```

然后输入sudo make install进行安装

```cpp
sudo make install
```

## 配置环境变量

进入 /etc/ld.so.conf.d/

```cpp
cd /etc/ld.so.conf.d/
```

创建 opencv.conf

```cpp
sudo touch opencv.conf
```

修改 opencv.conf

```cpp
sudo vi opencv.conf
```

输入/usr/local/lib

```cpp
/usr/local/lib
```

进入/etc/

```cpp
cd /etc/
```

修改bash.bashrc

```cpp
sudo vi bash.bashrc
```

在尾行输入：PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig export PKG_CONFIG_PATH

```cpp
PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig export PKG_CONFIG_PATH
```

