---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

---

# Mapping system

## Sensors
 - Velodyne PUCK VLP-16 
 - Velodyne ULTRAPUCK VLP-32c
 - Livo TELE-15
 - AHRS IMU
![Drawing](/assets/drawing.svg)

| Livox | VLP32c | VLP16  | All |
| - | - | - | - |
| [ <img src="assets/livox_full.gif" height="500"/> ](assets/livox_full.gif) |  [<img src="assets/vpl32c.gif" height="500"/>](assets/vpl32c.gif)  | [ <img src="assets/rot.gif" height="500"/>  ](assets/rot.gif)  | [ <img src="assets/all3_lasers.gif" height="500"/>  ](assets/all3_lasers.gif)  | 

## Time synchronization

Dataset is time synchronized using hardware synchronization. 
It is using artificial, generated PPS signal and custom acquisition software solution that allows to synchronization of incoming data streams effectively. 
Since this software solution is extremely hardware specific and requires custom-developed hardware, we decided to do not to release it with the dataset.
Feel free to contact us to get tips and recommendations for recreating such or a similar system.
Every point has an individual timestamp, synchronization precision is 1-2 milliseconds.

## Data acqusitiondistribution. You can use a newer version of ROS1 to work.

## Dataset description

- Compression : `BZ2`
- Raw data size : 49 Gb
- Length : 69 minutes (10 minutes per file)

### Topics

- /imu : `Sensor_msgs/IMU` 
- /livox :
`Sensor_msgs/Pointcloud2`
- /velodyne_rot :
`Sensor_msgs/Pointcloud2`
- /velodyne :
`Sensor_msgs/Pointcloud2`
- /tf : `tf2_msgs/TFMessage`
- /tf_static : `tf2_msgs/TFMessage`


Note that used point type is `PointXYZIRT`, the header for PCL library can be found [here](https://github.com/michalpelka/mine_dataset_process_bag/blob/master/point_types.h).

Data from AHRS IMU. ROS timestamp in header (also in '\tf' topic) is synchronized to rest of the system.

Sadly, there is no data recorder from IMU in Livox TELE-15

# Quick start with data

Files can be visualized using RViz, we provide custom, updated calibration to overide deafult one from rosbag.

Sample of usage:
In first terminal start roscore
```
rosparam set use_sim_time true
roscore
```
In second terminal start broadcasting new calibration and play bags files:
```
python mine-mapping-dataset/visualization/publish_custom_calibration.py &
rosbag play day1/mine_mapping_trajectory.bag day1/mine_mapping_0**.bag --clock -s 200 tf_static:=old_tf_static
```
In the third terminal open rviz:
```
rviz -d mine-mapping-dataset/visualization/config.rviz
```
Expected results:

[![YouTube](https://img.youtube.com/vi/A6Is-ao9THQ/0.jpg)](https://www.youtube.com/watch?v=A6Is-ao9THQ "YouTube")

[Youtube video](https://www.youtube.com/watch?v=A6Is-ao9THQ)

# Ground truth data
The environment was mapped with FARO Focus 3D and scans were assembled using geodetic targets.
The data is available here: https://drive.google.com/drive/folders/1kvlYNoj0QT_D8M65nwzR8D8I0pIArhMj?usp=sharing
The location of scans and targets is shown here.  
[ ![](assets/all_3_small.png) ](assets/all_3.png)  
The ground truth data is georeferenced in [PL2000](https://epsg.io/2178) coordinate system. 

The files avialable in groundtruth dataset:
 - `aI_processed.laz` First level FARO FOCUS 3D
 - `aIV_processed.laz` Fourth level FARO FOCUS 3D
 - `aVI_processed.laz` Sixth level FARO FOCUS 3D
 - `aV_processed.laz` Fifth level FARO FOCUS 3D
 - `aVIII_processed.laz` Eigth level FARO FOCUS 3D
 - `azrab_processed.laz` Enter level FARO FOCUS 3D
 - `livox_*****_processed.laz` - Reference data from mobile mappng system in [PL2000](https://epsg.io/2178) coordinate system. 
 - `crosses_with_extrapoint` - Locations of geodetic targets.
 - `result_pc_dec0_5.laz` - decimated pointcloud containing all data from  FARO FOCUS 3D
The names and location of targets in PL2000 coordinate system is given:

|target| X | Y | Z                        |
|------|------------|-----------|---------|
| TS-1 | -86345.352 | 22671.020 | 249.098 |
| TS-2 | -86346.390 | 22672.932 | 249.391 |
| TS-3 | -86347.665 | 22669.905 | 249.503 |
| TS-4 | -86347.457 | 22671.858 | 248.701 |
| T1-1 | -86347.239 | 22668.482 | 196.697 |
| T1-2 | -86347.484 | 22671.017 | 196.918 |
| T4-1 | -86346.082 | 22672.304 | 75.892  |
| T4-2 | -86345.222 | 22673.541 | 74.974  |
| T6-1 | -86345.387 | 22673.960 | 8.564   |
| T6-2 | -86344.347 | 22674.161 | 8.564   |
| T8-1 | -86345.550 | 22678.609 | -41.371 |
| T8-2 | -86343.015 | 22680.587 | -41.152 |
| T8-4 | -86347.258 | 22671.981 | -40.004 |
| T8-5 | -86347.847 | 22670.668 | -38.436 |
| T8-6 | -86346.088 | 22669.629 | -38.438 |

# Download
Current links to download rosbags from mobile system are avaialable here: 
[links.md](https://github.com/michalpelka/mine-mapping-dataset/blob/gh-pages/docs/assets/links.md)
The georeferenced ground truth is available here:
[Google Drive](https://drive.google.com/drive/folders/1kvlYNoj0QT_D8M65nwzR8D8I0pIArhMj?usp=sharing)

# Gallery
[ ![](assets/photo1_small.jpg) ](assets/photo1.jpg)
[ ![](assets/photo2_small.jpg) ](assets/photo2.jpg)
[ ![](assets/photo3_small.jpg) ](assets/photo3.jpg)