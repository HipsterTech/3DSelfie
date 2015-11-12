3DSelfie
=========================== 
3DSelfie is an attempt to make dense reconstruction using RGB-D frames, making use of PCL. In order to gain some additional insight of functionalities within PCL, a number of auxiliary tools are being developed. So far the following tools were developed:
* [ht_icp_check](https://github.com/HipsterTech/3DSelfie/tree/master/src/tools/ht_icp_check)
* [ht_normal_check](https://github.com/HipsterTech/3DSelfie/tree/master/src/tools/ht_normal_check)
* [ht_voxel_grid_check](https://github.com/HipsterTech/3DSelfie/tree/master/src/tools/ht_normal_check)
 
Although this project is cross-platform, we currently have only tested things under Darwin platforms, so please excuse us if it raises issues on other platforms.  

F. A. Q.
--------
*Q*: Why don't you simply use kinfu?

*A*: We don't have the hardware requirements for it :')

Compile Instructions
--------------------
```
$ git clone 
$ cd 3DSelfie
$ mkdir build
$ cd build/
$ cmake ..
```
The built binaries should be under ```3DSelfie/build/bin/```. Installation has not been tested at this point, so do it at your own risk

Dependencies
--------------------
3DSelfie
* [PCL (trunk) with OpenNI](https://github.com/PointCloudLibrary/pcl)
* [OpenCV 3](http://opencv.org/)
* [xfeatures2d for OpenCV 3](https://github.com/Itseez/opencv_contrib/tree/master/modules/xfeatures2d)

For depencies on each individual tool, check their own ```readme.md``` file.

TODO
--------------------
[ ] Too many things
[ ] Remove OpenCV dependency

General Info
--------------------
Maintained by Hipster Tech [@CarlosCrespog](https://github.com/carloscrespog) & [@SergioRAgostinho](https://github.com/SergioRAgostinho). Making cool things that really don't matter.

