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

Use the launch file to start the node:

```bash
    roslaunch baumer_ox_scanner pointcloud.launch

    roslaunch baumer_ox_scanner pointcloud_ee.launch # including transform to end effector
```

Or just run the node:

```bash
    rosrun baumer_ox_scanner scanner_pointcloud
```

For PointCloud2:

```bash
    rosrun baumer_ox_scanner scanner_pointcloud2
```

### Transformations

Transform PointCloud to PointCloud2:

```bash
    rosrun baumer_ox_scanner pointCloudToPointCloud2
```

Transform PointCloud2 into base_frame (output topic is /cloud_out):

```bash
    rosrun baumer_ox_scanner transform_cloud_publisher.py
```

### Republish data

To simulate continous data for the accumaulated pointcloud, the cloud is republished at 50Hz.
Default: sub_topic='/cloud_accumulated', pub_topic='cloud_high_rate'

```bash
    rosrun baumer_ox_scanner fake_cloud_high_rate_publisher.py
```

### Assemble pointClouds stream into one PointCloud

#### From bag file

Creates a txt file of all points with "x,y,z" in each line.
Transforms all points by transformation given via pose topic (default: "/robot_pose").

```bash
    python3 scripts/assemble_cloud_from_bag.py
```

#### Live

Launch the assembler service:

```bash
    roslaunch baumer_ox_scanner pointcloud_assembler.launch
```

Call the service to assemble the pointClouds and publish the data as floats. Should be run after the scanner is finished (Uses the buffer set in pointcloud_assembler.launch)

```bash
    rosrun baumer_ox_scanner assemble_cloud.py
```

For publishing the assembled pointCloud as PointCloud2 instead of floats:

```bash
rosrun baumer_ox_scanner assemble_pub_cloud
```

## Problem Solving <a name = "problems"></a>

### Could not connect to host

This could be due to not closing the WebInterface or another instance of the node running and connected to the scanner.
