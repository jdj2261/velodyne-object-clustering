# velodyne clustering

This repository can be used in an environment where ROS is not installed.

This code is specific to VLP-16, but it's very easy to adapt the code to HDL-32 or HDL-64E.

I have modified the velodyne driver supplied by the [this link](https://github.com/linbaiwpi/VLP16_driver_on_PYNQ) 



## How to use

~~~
$ mkdir build && cd build
$ cmake ..
$ make
$ cp -r test.pcap .
$ ./velodyne_cluster
~~~



## TODO

**[ ]** make it compatible Socket or PCAP 

**[ ]** Velodyne Viewer

**[ ]** Clustering (make Box)

