# image2pc
This code converts a depth image provided by the [pc2image](https://github.com/EPVelasco/pc2image/tree/depthImage_zImage) package of a Velodyne VLP16 3D-Lidar sensor into a point cloud.

## Requisites
- [pc2image](https://github.com/EPVelasco/pc2image/tree/depthImage_zImage)
- [ROS](http://wiki.ros.org/ROS/Installation) Kinetic or Melodic
- [Velodyne](https://github.com/ros-drivers/velodyne) repository
  ```
  sudo apt-get install ros-melodic-velodyne-pointcloud
  ```
- [PCL](https://pointclouds.org/) (Point Cloud Library)

## Clone repository
```
    cd ~/catkin_ws/src
    git clone https://github.com/EPVelasco/image2pc.git
    cd ..
    catkin_make --only-pkg-with-deps image2pc
```
## Ros run
```
    rosrun image2pc image2pc_node
```
