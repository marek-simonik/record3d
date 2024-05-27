# [*Record3D – Point Cloud Animation and Streaming*](https://record3d.app/): the accompanying library 

**2024/05/27 Update**: Added confidence map streaming (introduced breaking changes). **To be used with Record3D 1.10 and newer.**

**2022/08/16 Update**: Added camera position streaming (introduced breaking changes). **To be used with Record3D 1.7.2 and newer.**

**2021/07/28 Update**: Introduced support for higher-quality RGB LiDAR streaming. **To be used with Record3D 1.6 and newer.**

**2020/09/17 Update**: Introduced LiDAR support. To be used with Record3D 1.4 and newer.

This project provides C++ and Python libraries for the [iOS Record3D app](https://record3d.app/) which allows you (among other features) to 
live-stream RGB**D** video from iOS devices with TrueDepth camera to a computer via USB cable.

## Prerequisites
  - Install [CMake](https://cmake.org/download/) >= **3.13.0** and make sure it is in `PATH`.
  - When on macOS and Windows, install [iTunes](https://www.apple.com/itunes/).
  - When on Linux, install [`libusbmuxd`](https://launchpad.net/ubuntu/+source/libusbmuxd) (`sudo apt install libusbmuxd-dev`). It should be installed by default on Ubuntu.
  
## Installing
The libraries are multiplatform — macOS, Linux and Windows are supported.

### Python
You can install either via `pip`:
    
    python -m pip install record3d

or build from source (run as admin/root):
    
    git clone https://github.com/marek-simonik/record3d
    cd record3d
    python setup.py install
    
### C++
After running the following, you will find compiled static library in the `build` folder and header files in the `include` folder.

**macOS and Linux**

    git clone https://github.com/marek-simonik/record3d
    cd record3d
    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make -j8 record3d_cpp
    make install
    # now you can link against the `record3d_cpp` library in your project

**Windows**

    git clone https://github.com/marek-simonik/record3d
    cd record3d
    md build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release -G "Visual Studio 15 2017 Win64" ..
    # Open the generated Visual Studio Solution (.sln) fil and build the "`record3d_cpp`" Project

## Sample applications
There is a Python (`demo-main.py`) and C++ (`src/DemoMain.cpp`) sample project that demonstrates how to use the library to receive and display RGBD stream.

Before running the sample applications, connect your iOS device to your computer and open the Record3D iOS app. Go to the Settings tab and enable "USB Streaming mode".

### Python
After installing the `record3d` library, run `python demo-main.py` and press the record button to start streaming RGBD data.

### C++
You can build the C++ demo app by running the following (press the record button in the iOS app to start streaming RGBD data):

**macOS and Linux**

    git clone https://github.com/marek-simonik/record3d
    cd record3d
    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make -j8 demo
    ./demo
    
**Windows**

    git clone https://github.com/marek-simonik/record3d
    cd record3d
    md build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release -G "Visual Studio 15 2017 Win64" ..
    # Open the generated Visual Studio Solution (.sln) file and run the "`demo`" Project
