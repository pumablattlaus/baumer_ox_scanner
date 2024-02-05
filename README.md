# Project Title

## Table of Contents

- [About](#about)
- [Getting Started](#getting_started)
- [Usage](#usage)
- [Problem Solving](#problems)

## About <a name = "about"></a>

ROS Package for Baumer OXP200 scanner unsing C++ API. Publishes a PointCloud in frame "laser_scanner".

## Getting Started <a name = "getting_started"></a>

1. Set up the scanner via WebInterface. Use Height Mode instead of Distance Mode.
2. clone package into catkin_ws/src
3. catkin build
4. source devel/setup.bash


### Prerequisites

ROS Noetic has to be set up.

### Installing
#### C++ API for ubuntu

**API already part of package**. In case you want to install it from source (Installing from LibOxApi_02-00-00.tar.gz):

```bash
    tar -xvzf LibOxApi_02-00-00.tar.gz
    cd LibOxApi_02-00-00
    sudo cp -ri include/* /usr/local/include/
    sudo cp -ri lib/* /usr/local/lib/
```

Add lib to path (or in bashrc):

```bash
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
```

## Usage <a name = "usage"></a>

Just run the node:

```bash
    rosrun baumer_ox_scanner scanner_pointcloud
```

For PointCloud2:

```bash
    rosrun baumer_ox_scanner scanner_pointcloud2
```
*Pointcloud2 is not yet tested*

## Problem Solving <a name = "problems"></a>

### Could not connect to host

This could be due to not closing the WebInterface or another instance of the node running and connected to the scanner.
