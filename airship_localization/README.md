# airship_localization
**This project integrates IMU sensor and LiDAR sensor to achieve mapping and localization of robot. Current V0.0 localization modules are depend on Cartographer project for fast deployment. Some adjustments are maked for coverting LiDAR sensen message data type into Cartographer inter data type. We also provide the offline map in "/map", for your own data, please follow the map saving instruction below and replace the "load_state_filename" address in ros launch command.**

![](./doc/demo.gif)

## airship_localization API Overview
### rslidar_conversion_node

1. Subscribed Topics
- `topic_sub_rslidar_points`: Subscribes to RoboSense LiDAR sensor msg. (Default `/rslidar_points`)

2. Published Topics
- `topic_pub_rslidar_points_converted`: Publishes PointCloud msg converted into Cartorgrapher type. (Default `/rslidar_points_converted`) 

#### Sensor msg callback

- In "rslidar_conversion_node.cpp", we convert the rslidar sensor msg into cartographer pointcloud data type. Please check the code if you are intend to run on your own data.
```cpp
namespace rslidar_ros{
  struct EIGEN_ALIGN16 Point {
      float x;
      float y;
      float z;
      float intensity;
      uint16_t ring = 0;
      double timestamp = 0;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace rslidar_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(rslidar_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint16_t, ring, ring)
    (double, timestamp, timestamp)
)

namespace cartographer{
  struct EIGEN_ALIGN16 PointXYZIT {
      float x;
      float y;
      float z;
      float intensity;
      float time;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace cartographer
POINT_CLOUD_REGISTER_POINT_STRUCT(cartographer::PointXYZIT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
)
```
### cartographer_node

- Please refer to cartographer official document, [link](https://google-cartographer-ros.readthedocs.io/en/latest/).

## Example for Map Saving
```shell
# 1, start a new termial, launch cartographer mapping node
ros2 launch airship_localization airship_mapping_2d.launch.py bag_filename:= address_to _bag.db3

# 2, start a new termial, if the mapping is finished, call finish_trajectory
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"

# 3, start a new termial, call write_state
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: 'address for saving/map.pbstream'}"
```

## To Do List

- [ ] V1.0 is currently under development. It contains two main parts, SLAM node and Map node. 
- [ ] In SLAM node, we are focus on the basic aspects, such as realtime performance, localization accuracy and stability. LIO/LVIO odometry, factor graph backend, relocalization method, BA optimization will be provided in SLAM node.
- [ ] Map node is designed for robot level application, including grid map for trajectory planning, 3DGS for sence reconstruction, semantic map for task decision making.
