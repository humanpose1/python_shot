# SHOT descriptor  for 3D Point Cloud in python


This repo is a binding in python  of the [SHOT descriptor](http://vision.disi.unibo.it/research/80-shot) written in C++ with [PCL](https://pointclouds.org/). It uses the library [pybind11](https://github.com/pybind/pybind11) for the binding.

## Installation

First you need to install [PCL](https://pointclouds.org/). You can use the script (thanls for [3DSmoothnet](https://github.com/zgojcic/3DSmoothNet) for the script) 
```
sh install_pcl.sh
```
then create a conda environnement
```
conda create -n "python_shot"
```
and install numpy and pytest
```
conda install numpy, pytest
```
Then you can install our library to compute shot
```
python setup.py install
```
## How to use SHOT descriptor

The SHOT function takes numpy array as argument. The arguments
```python
import handcrafted_descriptor as hd
descr = hd.compute_shot(point_cloud, normals, keypoints, keypoints_normal)
```
`point_cloud` is a numpy array of size `N x 3`
`normals` is a numpy array of size `N x 3`. It is the normals of the point cloud, you need to estimate it beforehand if you do not have access to normals.


`keypoints` is a numpy array of size `M x 3`. It is selected keypoints where you want to compute the descriptors.
`keypoints_normals` is a numpy array of size `M x 3`. the normals of the keypoints.

`descr` is a numpy array of size `M x 352`

