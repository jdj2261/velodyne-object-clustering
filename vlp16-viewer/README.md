# velodyne clustering

This repository can be used in an environment where ROS is not installed.

This code is specific to VLP-16, but it's very easy to adapt the code to HDL-32 or HDL-64E.

I have modified the velodyne driver supplied by the [this link](https://github.com/linbaiwpi/VLP16_driver_on_PYNQ) 



## How to use

~~~shell
$ mkdir build && cd build
$ cmake ..
$ make
$ cd tests
$ ./ViewerTest or ./ViewerTest -pcap test.pcap
~~~

- socket example

![socket](https://user-images.githubusercontent.com/35681273/118909963-c0f0c100-b95e-11eb-9876-e28f32bec3f6.gif)

- pcap example

![pcap](https://user-images.githubusercontent.com/35681273/118909951-bcc4a380-b95e-11eb-855c-e2e9346d7185.gif)



## TODO

**[O]** make it compatible Socket or PCAP 

**[O]** Velodyne Viewer

**[ ]** Clustering (make Box)

