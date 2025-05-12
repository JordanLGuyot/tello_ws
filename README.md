# tello_ws
MTRX5700: Tello driver for ROS2 Humble.
- [DJI Tello](https://www.ryzerobotics.com/tello) driver for ROS 2 based on [DJITelloPy](https://github.com/damiafuentes/DJITelloPy) that uses the [official SDK](https://github.com/dji-sdk/Tello-Python) for the drone.
- It is recommended to update the Tello firmware to the latest version available 
- Project workspace is divided into sub-workspaces that contain different logic.
  - `tello` package is the main package, includes access to the drone information, camera image and  control.
  - `tello_msg` package defines custom messages to access specific Tello data.
    - Defines the `TelloStatus`, `TelloID` and `TelloWifiConfig` messages 
  - `tello_control` package is a sample control package that displays the drone image and provides keyboard control.
    - T used for takeoff, L to land the drone, F to flip forward, E for emergency stop, WASD and arrows to control the drone movement.

## Tello Package
- Bellow is the list of topics published and consumed by the `tello` package
- The list of published topics alongside their description and frequency. These topics are only published when some node subscribed to them, otherwise they are not processed.

| Topic        | Type                           | Description                                                  | Frequency |
| ------------ | ------------------------------ | ------------------------------------------------------------ | --------- |
| /image_raw   | sensor_msgs/Image              | Image of the Tello camera                                    | 30hz      |
| /camera_info | sensor_msgs/CameraInfo         | Camera information (size, calibration, etc)                  | 2hz       |
| /status      | tello_msg/TelloStatus          | Status of the drone (wifi strength, batery, temperature, etc) | 2hz       |
| /id          | tello_msg/TelloID              | Identification of the drone w/ serial number and firmware    | 2hz       |
| /imu         | sensor_msgs/Imu                | Imu data capture from the drone                              | 10hz      |
| /battery     | sensor_msgs/BatteryState       | Battery status                                               | 2hz       |
| /temperature | sensor_msgs/Temperature        | Temperature of the drone                                     | 2hz       |
| /odom        | nav_msgs/Odometry              | Odometry (only orientation and speed)                        | 10hz      |
| /tf          | geometry_msgs/TransformStamped | Transform from base to drone tf, prefer a external publisher. | 10hz      |

- The list of topics subscribed by the node, these topics can be renamed in the launch file.

| Topic        | Type                      | Description                                                  |
| ------------ | ------------------------- | ------------------------------------------------------------ |
| \emergency   | std_msgs/Empty            | When received the drone instantly shuts its motors off (even when flying), used for safety purposes |
| \takeoff     | std_msgs/Empty            | Drone takeoff message, make sure that the drone has space to takeoff safely before usage. |
| \land        | std_msgs/Empty            | Land the drone.                                              |
| \control     | geometry_msgs/Twist       | Control the drone analogically. Linear values should range from -100 to 100, speed can be set in x, y, z for movement in 3D space. Angular rotation is performed in the z coordinate. Coordinates are relative to the drone position (x always relative to the direction of the drone) |
| \flip        | std_msgs/String           | Do a flip with the drone in a direction specified. Possible directions can be "r" for right, "l" for left, "f" for forward or "b" for backward. |
| \wifi_config | tello_msg/TelloWifiConfig | Configure the wifi credential that should be used by the drone. The drone will be restarted after the credentials are changed. |

- The list of parameters used to configure the node. These should be defined on a launch file.

| Name             | Type    | Description                                                  | Default        |
| ---------------- | ------- | ------------------------------------------------------------ | -------------- |
| connect_timeout  | float   | Time  (seconds) until the node is killed if connection to the drone is not available. | 10.0           |
| tello_ip         | string  | IP of the tello drone. When using multiple drones multiple nodes with different IP can be launched. | '192.168.10.1' |
| tf_base          | string  | Base tf to be used when publishing data                      | 'map'          |
| tf_drone         | string  | Name of the drone tf to use when publishing data             | 'drone'        |
| tf_pub           | boolean | If true a static TF from tf_base to tf_drone is published    | False          |
| camera_info_file | string  | Path to a YAML camera calibration file (obtained with the calibration tool) | ''             |

### To run Tello Driver:
1. Build the workspace containing the particular packages, from the base directory of GitHub:
```bash
colcon build --packages-select tello tello_msg --symlink-install
```
2. Source the workspace:
```bash
source install/setup.bash
```
3. Run the launch file for the driver:
```bash
ros2 run tello tello --ros-args -p tello_ip:=192.168.10.1 -r image_raw:=camera
```