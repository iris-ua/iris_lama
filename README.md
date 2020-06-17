LaMa - A Localization and Mapping library.
==========================================
https://github.com/iris-ua/iris_lama

![Build](https://github.com/iris-ua/iris_lama/workflows/Build/badge.svg)

Developed and maintained by Eurico Pedrosa, University of Aveiro (C) 2019.

Overview
--------
LaMa is a C++11 software library for robotic localization and mapping developed at the **Intelligent Robotics and Systems** (IRIS) Laboratory from the University of Aveiro - Portugal. It includes a framework for 3D volumetric grids (for mapping), a localization algorithm based on scan matching and two SLAM solution (an *Online SLAM* and a *Particle Filter SLAM*).

The main feature is *efficiency*. Low computational effort and low memory usage whenever possible. The minimum viable computer to run our localization and SLAM solutions is a [Raspberry Pi 3 Model B+](https://www.raspberrypi.org/products/raspberry-pi-3-model-b-plus/).

#### Build

To build LaMa, clone it from GitHub and use CMake to build.
```
$ git clone https://github.com/iris-ua/iris_lama
$ cd iris_lama
$ mkdir build
$ cd build
$ cmake ..
```
Its only dependency is [Eigen3](http://eigen.tuxfamily.org).
**Note**: LaMa does not provide any executable. For an example on how to use it, please take a look at our integration with ROS.

#### Integration with ROS 

The source code contains `package.xml` so that it can be used as a library from external ros packages.
We provide ROS nodes to run the localization and the two SLAM solutions. Please go to [iris_lama_ros](https://github.com/iris-ua/iris_lama_ros) for more information.


Sparse-Dense Mapping (SDM)
--------------------------
Sparse-Dense Mapping (SDM) is a framework for efficient implementation of 3D volumetric grids. Its divides space into small dense *patches* addressable by a sparse data-structure. To improve memory usage each individual *patch* can be compressed during live operations using lossless data compression (currently [lz4](https://github.com/lz4/lz4) and [Zstandard](https://github.com/facebook/zstd)) with *low overhead*.
It can be a replacement for [OctoMap](https://octomap.github.io/).

Currently it has the following grid maps implemented:
* **Distance Map**: It provides the distance to the closest occupied cells in the map. We provide the `DynamicDistanceMap` which is an implementation of the dynamic Euclidean map proposed by:
    > B. Lau, C. Sprunk, and W. Burgard 
    > **Efficient Grid-Based Spatial Representations for Robot Navigation in Dynamic Environments**
    > Robotics and Autonomous Systems, 61 (10), 2013, pp. 1116-1130, Elsevier

* **Occupancy Map**: The most common representation of the environment used in robotics. Three (3) variants of the occupancy map are provided: a `SimpleOccupancyMap` where each cell has a tri-state: *free*, *occupied* or *unknown*: a `ProbabilisticOccupancyMap` that encodes the occupancy probability of each cell with logods; and a `FrequencyOccupancyMap` that tracks the number of times a beam hits or traverses (miss) a cell and calculates a hit/miss ratio.


For more information about **SDM** please read
> Eurico Pedrosa, Artur Pereira, Nuno Lau\
> **A Sparse-Dense Approach for Efficient Grid Mapping**\
> 2018 IEEE International Conference on Autonomous Robot Systems and Competitions (ICARSC)

Localization based on Scan Matching
-----------------------------------
We provide a **fast** scan matching approach to mobile robot localization supported by a continuous likelihood field. It can be used to provide accurate localization for robots equipped with a laser and a *not so good* odometry. Nevertheless, a good odometry is always recommended.

> Eurico Pedrosa, Artur Pereira, Nuno Lau\
> **Efficient Localization Based on Scan Matching with a Continuous Likelihood Field**\
> 2017 IEEE International Conference on Autonomous Robot Systems and Competitions (ICARSC)

Online SLAM
-----------

For environments without considerable loops this solution can be accurate and very efficient. It can run in *real time* even on a low-spec computer (we have it running on a turtlebot with a raspberry pi 3B+). It uses our localization algorithm combined with a dynamic likelihood field to incrementally build an occupancy map.

For more information please read
> Eurico Pedrosa, Artur Pereira, Nuno Lau\
> **A Non-Linear Least Squares Approach to SLAM using a Dynamic Likelihood Field**\
> Journal of Intelligent & Robotic Systems 93 (3-4), 519-532

Multi-threaded Particle Filter SLAM
--------------------

This Particle Filter SLAM is a RBPF SLAM like [GMapping](https://openslam-org.github.io/) and it is the extension of the Online SLAM solution to multiple particles with multi-thread support. Our solution is capable of parallelizing both the localization and mapping processes. It uses a thread-pool to manage the number of working threads.

Even without multi-threading, our solutions is a lightweight competitor against the heavyweight [GMapping](https://openslam-org.github.io/).

For more information please read
>Eurico Pedrosa, Artur Pereira, Nuno Lau\
> **Fast Grid SLAM Based on Particle Filter with Scan Matching and Multithreading**\
> 2020 IEEE International Conference on Autonomous Robot Systems and Competitions (ICARSC), Ponta Delgada, Portugal, 2020, pp. 194-199, doi: 10.1109/ICARSC49921.2020.9096191.

