# Large Point Cloud Preprocessing
## Overview
**Author: [Igor Maurell], igormaurell@gmail.com**

The large_point_cloud_preprocessing software uses PCL to preprocess a point cloud, doing:
### Filtering
- Cut-off
- Voxel Grid
- Statistical Outlier Removal
### Normal Estimation
- Normal Estimation OMP
### Normalization
- Reescale
- Centralization
- Alignment
- Noise Add
- Cube Reescale

## Dependencies
This software is implemented in C++14 using the [Point Cloud Library](https://pointclouds.org) (PCL-1.10).

## Instalation
After install PCL-1.10, do:
    
    $ git clone --recursive https://github.com/igormaurell/large_point_cloud_preprocessing
    $ cd large_point_cloud_processing
    $ mkdir build && cd build
    $ cmake ..
    $ make

## Using
After install, use the help to understand the parameters, doing:
    
    $ ./large_point_cloud_preprocessing -h