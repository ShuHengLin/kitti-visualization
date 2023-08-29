# kitti-visualization

## Prepare Data
[3D Object Detection Evaluation 2017](http://www.cvlibs.net/datasets/kitti/eval_object.php?obj_benchmark=3d)
* **data_object_image_2.zip** → Download left color images of object data set (12 GB)
* **data_object_calib.zip** &emsp;&ensp; → Download camera calibration matrices of object data set (16 MB)
* **data_object_label_2.zip** &ensp; → Download training labels of object data set (5 MB)
* **data_object_velodyne.zip** → DDownload Velodyne point clouds, if you want to use laser information (29 GB)


## Visualize lidar pointcloud
* Using rviz to visualize:
```
roscore
```
```
rosrun rviz rviz -d rviz_config.rviz
```
```
python -B vis_lidar.py
```


## Visualize camera image
```
python -B vis_camera.py
```


## The KITTI label format
| type | truncated | occluded | alpha | bbox | dimensions | location | rotation |
| ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- |
| Car | 0.0 | 0 | 1.84 | 390.29 &thinsp; 181.00 &thinsp; 423.13 &thinsp; 201.74 | 1.47 &thinsp; 1.62 &thinsp; 4.07 | -16.19 &thinsp; 2.21 &thinsp; 57.39 | 1.57 |


## References
1) [Are we ready for Autonomous Driving? The KITTI Vision Benchmark Suite](https://projet.liris.cnrs.fr/imagine/pub/proceedings/CVPR2012/data/papers/424_O3C-04.pdf)
2) [Vision meets Robotics: The KITTI Dataset](https://www.cvlibs.net/publications/Geiger2013IJRR.pdf)
3) [Convert to KITTI Format for Evaluation #23](https://github.com/skhadem/3D-BoundingBox/issues/23)
