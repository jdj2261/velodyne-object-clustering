velodyne-object-clustering
===============

This package is that allows you to view the data of velodyne vlp-16 sensor using pcl library in ubuntu 18.04 environment. In addition, you can check the object clustering algorithm and results.


Environments
-----------
* ubuntu 18.04
* g++ 8.4.0
* CMake 3.10
* pcl 1.7


Components
------
This repository has three sample programs.  

- simple

  This program displays data on standard output using VelodyneCapture class.

- velodyne_viewer

  It is an object detection program created based on the above code.

- vlp16-viewer

  It is an object detection program based on the ROS Velodyne driver package.

## TODO

**[O]** Velodyne Viewer

**[O]** Clustering (make Box)

**[   ]** UKF Algorithm

