ht_icp_check 
=============
A tool meant to provide quick visualization of the outcome of ICP between two point clouds. 
Dependencies
------------
* [PCL trunk](https://github.com/PointCloudLibrary/pcl)
* [boost-program-options](http://www.boost.org/)  (will be removed soon)


Examples
--------
A few examples are shown below.
#### Print help
```
$ ./ht_icp_check  --help
Usage: ht_icp_check file1 [file2] [options]
Allowed Options:
  --help                                produce help message
  --input-file arg                      input file(s) with point cloud
  --loop arg (=0)                       set loop period t in s. If t equals 0, 
                                        no loop takes place.
  --max-dist arg (=1.3407807929942596e+154)
                                        maximum correspondence distance
  --rotate arg                          rotation to be applied to second point 
                                        cloud before icp. Rotation of angle 
                                        (degrees) over vector v. --rotate vx vy
                                        vz angled
  --translate arg                       translation to be applied to second 
                                        point cloud before icp. --translate tx 
                                        ty tz
```

#### Align the same point cloud
```
$ ./ht_icp_check  3DSelfie/data/suzanne_monkey_high.ply
```
```suzanne_monkey_high.ply``` will be the target point cloud.

A copy of ```suzanne_monkey_high.ply``` will be the input point cloud.

Not really a useful example by itself, if no transformation is specified.


#### Align two point clouds
```
$ ./ht_icp_check  3DSelfie/data/suzanne_monkey_high.ply 3DSelfie/data/suzanne_monkey_right_high.ply 
```
```suzanne_monkey_high.ply``` will be the target point cloud.

```suzanne_monkey_right_high.ply``` will be the input point cloud.

#### Specify maximum correspondence distance
```
$ ./ht_icp_check  3DSelfie/data/suzanne_monkey_left_high.ply 3DSelfie/data/suzanne_monkey_right_high.ply --max-dist 0.01
```
```suzanne_monkey_left_high.ply``` will be the target point cloud.

```suzanne_monkey_right_high.ply``` will be the input point cloud.

The maximum correspondence distance used will be ```0.01```.

#### Apply transformations to the input cloud
```
$ ./ht_icp_check  3DSelfie/data/suzanne_monkey_high.ply --translate 0.4 0 0
```
```suzanne_monkey_high.ply``` will be the target point cloud.

A copy of ```suzanne_monkey_high.ply``` will be the input point cloud.

The input cloud will be translated along the x axis.

```
$ ./ht_icp_check  3DSelfie/data/suzanne_monkey_high.ply --rotate 0 1 0 90
```
```suzanne_monkey_high.ply``` will be the target point cloud.

A copy of ```suzanne_monkey_high.ply``` will be the input point cloud.

The input cloud will be rotated 90 degrees along the y axis

#### Reset the alignment every x seconds
```
$ ./ht_icp_check  3DSelfie/data/suzanne_monkey_high.ply --translate 0.4 0 0 --loop 10
```
```suzanne_monkey_high.ply``` will be the target point cloud.

A copy of ```suzanne_monkey_high.ply``` will be the input point cloud.

The input cloud will be translated along the x axis and the alignment will be reset every 10 seconds, as soon as icp returns from its current align attempt. 



watch magic happen... really slowly.

### TODOs

- [x] Implement proper command line parsing (Unix style)
- [ ] Print all info in the GUI
- [ ] Remove the depency from boost-program-options and use pcl::console

