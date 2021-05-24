# vlp16-viewer

This directory can be used in an environment where ROS is not installed.

This code is specific to VLP-16, but it's very easy to adapt the code to HDL-32 or HDL-64E.

I have modified the velodyne driver supplied by the [this link](https://github.com/linbaiwpi/VLP16_driver_on_PYNQ) 

## How to use

### 1. vlp16_viewer

> Write down the -h option to see how to use it.

~~~shell
$ mkdir build && cd build
$ cmake ..
$ make
$ cd tests
$ ./ViewerTest or ./ViewerTest -pcap test.pcap
~~~

- socket example

~~~
$ ./ViewerTest
~~~

![socket](https://user-images.githubusercontent.com/35681273/118909963-c0f0c100-b95e-11eb-9876-e28f32bec3f6.gif)

- pcap example

~~~shell
$ ./ViewerTest -pcap {pcap file path}
example >>> $ ./ViewerTest -pcap test.pcap
~~~

![pcap](https://user-images.githubusercontent.com/35681273/118909951-bcc4a380-b95e-11eb-855c-e2e9346d7185.gif)

### 2. vlp16_cluster

This is an example of clustering with the same pcap file as the example above.

The picture below was taken at Sejong City Central Park. I got the vlp-16 pcap file from here.

<left><img src="doc/sejong.jpg" style="zoom: 10%"></left>

- cluster example

~~~
$ mkdir build && cd build
$ cmake ..
$ make
$ cd tests
$ ./renderTest or ./renderTest -pcap test.pcap
~~~

![clustering](https://user-images.githubusercontent.com/35681273/119290934-82be0f00-bc88-11eb-9111-c28ed90e5644.gif)

## TODO

**[O]** make it compatible Socket or PCAP 

**[O]** Velodyne Viewer

**[O]** Clustering (make Box)

**[ - ]** MultiThreading 

