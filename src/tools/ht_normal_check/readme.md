ht_normal_check 
=============
A tool meant to provide quick visualization of the outcome of normal estimation on a point cloud. This tool makes use of the OpenMP implementation of normal estimation. For that, compile PCL with OpenMP support. 

Dependencies
------------
* [PCL trunk](https://github.com/PointCloudLibrary/pcl)

Examples
--------
#### Print help
```
$ ./ht_normal_check  -h
Usage: ht_voxel_grid <file> [options]
Options
-h,--help  prints this help message
-r radius    set the radius used for normal search
```

#### Estimate normals on a given pointcloud
```
$ ./ht_normal_check  3DSelfie/data/suzanne_monkey_high.ply
```
And wait... OpenMP helps a little bit. 


#### Specify the search radius
```
$ ./ht_normal_check  3DSelfie/data/suzanne_monkey_high.ply -r 1
```
The search radius to ```1```.


### TODOs

