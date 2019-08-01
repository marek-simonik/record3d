# libusbmuxd-win32
[![Build status](https://ci.appveyor.com/api/projects/status/pru5c9lgtx12q4xj?svg=true)](https://ci.appveyor.com/project/qmfrederik/libusbmuxd)
[![Build Status](https://travis-ci.org/libimobiledevice-win32/libusbmuxd.svg?branch=msvc-master)](https://travis-ci.org/libimobiledevice-win32/libusbmuxd)

Provides a native Windows build (using the Visual C++ compiler) of libusbmuxd, as well as continuous integration (CI) builds of libusbmuxd for Ubuntu, CentOS and RedHat Linux and macOS.

## Where to report issues
For general questions about libplist, see http://github.com/libimobiledevice/libusbmuxd. For questions specific to Visual C++, feel free to use the GitHub issue tracker

## How to get the latest binaries
The binaries for libplist are available as:
* [NuGet CoApp packages](https://www.nuget.org/packages/libusbmuxd/) for Windows,
* [apt-get packages](https://launchpad.net/~quamotion/+archive/ubuntu/ppa) for Ubuntu,
* [yum packages](https://build.opensuse.org/package/show/home:qmfrederik/libusbmuxd) for CentOS and RedHat.

For Ubuntu Linux, run the following commands as root:

```
sudo add-apt-repository ppa:quamotion/ppa
sudo apt-get update
apt-get install libusbmuxd
```

For RedHat Linux, run the following commands as root:

```
cd /etc/yum.repos.d/
wget http://download.opensuse.org/repositories/home:qmfrederik/RHEL_7/home:qmfrederik.repo
yum install libusbmuxd
```

For CentOS Linux, run the following commands as root:

```
cd /etc/yum.repos.d/
wget http://download.opensuse.org/repositories/home:qmfrederik/CentOS_7/home:qmfrederik.repo
yum install libusbmuxd
```