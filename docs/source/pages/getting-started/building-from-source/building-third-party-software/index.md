 
# Building third-party software

## Build the HighFive library

The HighFive library is used for simulation output serialization to Hierarchical
Data Format 5 (HDF5).

The HighFive library requires the HDF5 development headers to be installed.

```shell
apt-get install --yes libhdf5-dev
```

Clone the HighFive repository to a new folder:

```shell
git clone https://github.com/highfive-devs/highfive highfive
```

Create a `build` folder and configure the CMake project.

```shell
mkdir -p highfive/build
cd highfive/build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DHIGHFIVE_UNIT_TESTS=OFF
```

Run the build:

```shell
cmake --build . -j$(nproc) --target install && \
```

Install the HighFive library at system level:

```shell
cmake --build . --target install
```

## Building Project Chrono

This section will guide you through installing Project Chrono with the modules
required to run DYNO.

DYNO has been tested on the **Project Chrono** main branch, commit
`c5c97f2117baf67f5194c84a8f4afddf506f5d02`. The minimum set of modules required
to run DYNO are:

* Chrono::Vehicle
* Chrono::Irrlicht

In addition to the above, the following modules provide additional
functionalities in DYNO:

* Chrono::Sensor, for sensor simulation in the DYNO ROS 2 interface
* Chrono::VSG, for visualization using the VulkanSceneGraph engine

You may follow the [build guide](https://api.projectchrono.org/tutorial_install_chrono_linux.html) on the
official Project Chrono documentation  guide on the official Project Chrono
documentation for building the Project Chrono core and the Chrono::Vehicle
modules from sources, and the [VSG module build guide](https://api.chrono.projectchrono.org/module_vsg_installation.html) for the
Chrono::VSG module.

Install the dependencies:

```shell
  sudo apt install --yes \
    build-essential \
    cmake \
    cmake-curses-gui \
    libeigen3-dev
```

Clone the repository:

```shell
mkdir ~/Repositories/github.com/projectchrono
cd ~/Repositories/github.com/projectchrono
git clone https://github.com/projectchrono/chrono
cd chrono
git checkout feature/cvt_transmission
```

```shell
mkdir build
cd build
ccmake ../
```

Press [c] then [e] then [q]

Optionally, build OpenCRG:

```shell
sudo mkdir -p /opt/opencrg
sudo apt install --yes unzip
cd contrib/build-scripts/opencrg
chmod +x ./buildOpenCRG.sh
sudo ./buildOpenCRG.sh /opt/opencrg
sudo apt install --yes ninja-build
```

Optionally, build VulkanSceneGraph. First, install the dependencies for the OpenGL module:

```shell
sudo apt install --yes \
  libxcb1-dev \
  libxinerama-dev \
  libxcursor-dev \
  libxrandr-dev \
  libgl1-mesa-dev \
  freeglut3-dev
```
