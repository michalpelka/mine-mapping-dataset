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

## Data acqusition
Data was saved using [ROS](http://wiki.ros.org/rosbag) bag format using Melodic ROS distribution. You can use a newer version of ROS1 to work.

## Dataset description

- Compression : `BZ2`
- Raw data size : 225 Gb
- Length : 69 minutes (60 second per file)

### Topics

- /imu/data_hwts : `Sensor_msgs/IMU` 
- /livox_raw :
`Sensor_msgs/Pointcloud2`
- /velodyne_rot :
`Sensor_msgs/Pointcloud2`
- /velodyne_static_raw :
`Sensor_msgs/Pointcloud2`

Note that used point type is `PCL::PointXYZINormal`, and normal fields were repourpouse for other [metadata](https://github.com/michalpelka/mine-mapping-export-las/blob/main/process_mine_data/src/release_data_laz.cpp#L156):
- `NormalX` - hardware timestamp
- `NormalY` - rotation unit angle, revelant to `/velodyne_rot`
- `NormalZ` - ring number 

Data from AHRS IMU. ROS timestamp in header is synchronized to rest of the system.

Sadly, there is no data recorder from IMU in Livox TELE-15

# Quick start with data

As a starting point we provide you with a Docker image that enables you to generate LAZ files using dataset and trajectory from our SLAM solution. Please refer to [readme](https://github.com/michalpelka/mine-mapping-export-las/blob/main/README.md) to learn more.
 
# Download
Please provide your email to obtain a link to download this [questionnaire](https://forms.gle/Z3NiTrCbX5dHVExC7).

# Gallery
[ ![](assets/photo1_small.jpg) ](assets/photo1.jpg)
[ ![](assets/photo2_small.jpg) ](assets/photo2.jpg)
[ ![](assets/photo3_small.jpg) ](assets/photo3.jpg)