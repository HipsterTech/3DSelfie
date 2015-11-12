ht_voxel_grid_check 
=============
A tool meant to provide quick visualization of the outcome of applying voxel grid downsamplling on a point cloud. 

Dependencies
------------
* [PCL trunk](https://github.com/PointCloudLibrary/pcl)

Examples
--------
#### Print help
```
$ ./ht_voxel_grid_check  -h
Usage: ht_voxel_grid <file> [options]
Options
-h,--help	 prints this help message
-l lx,ly,lz 	 set the leaf size for voxel downsampling
```

#### Voxel decimate a given pointcloud
```
$ ./ht_voxel_grid_check  3DSelfie/data/suzanne_monkey_high.ply
```
Performs voxel decimation with the standard leaf size 0.01 x 0.01 x 0.01.
For suzanne_monkey_high.ply, this should result in a pointcloud similar to the original. 


#### Voxel decimate a given pointcloud specifying the leaf size
```
$ ./ht_voxel_grid_check  3DSelfie/data/suzanne_monkey_high.ply -l 0.05,0.05,0.05
```
Performs voxel decimation with the standard leaf size 0.05 x 0.05 x 0.05.


### TODOs
[ ] Allow saving the resultant pointcloud to a file. 