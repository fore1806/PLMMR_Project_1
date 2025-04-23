# First Project: Vehicle Pose Estimation and Sector Analysis on Monza Circuit

## Overview

This ROS1-based project (written in C++) analyzes the motion of a vehicle around the Monza racing circuit using data extracted from a bag file (provided by the professors). The project is structured around three ROS nodes that process GPS data, steering angles, and velocity readings to estimate the vehicle's pose and performance in different sectors of the track.

## Node Descriptions

### 🚗 odometer.cpp

This node estimates the pose of the vehicle using a simplified bicycle model based on the steering angle and velocity inputs. The pose is computed by integrating the vehicle's motion over time using the bicycle kinematics equations.

- **Inputs**: Steering angle and velocity (subscribed from ROS topics).
- **Model**: A fixed-parameter bicycle model with configurable parameters for wheelbase and steering factor.
- **Pose Estimation**: Calculates position updates using the current speed and angular change derived from steering. Orientation is handled with quaternions.
- **Features**:
  - Publishes `nav_msgs/Odometry` messages with custom covariance values.
  - Broadcasts TF transformations for RViz.
  - Publishes a `nav_msgs/Path` for debugging (optional).
  - Applies a basic smoothing filter to reduce noise in steering inputs.

### 🌍 gps_odometer.cpp

This node provides a more accurate estimate of the vehicle pose based on GPS data (latitude and longitude).

- **Inputs**: Latitude and longitude from a GPS topic.
- **Conversion**: Converts GPS data to ECEF coordinates, then translates them into a local reference frame (in meters).
- **Pose Estimation**: Calculates heading by comparing positions from two consecutive GPS points.
- **Advantages**: Offers higher precision than the bicycle model but is sensitive to GPS dropouts, which are handled by discarding inconsistent data.
- **Outputs**: Publishes odometry messages and TF transforms.

### ⏱️ sector_times.cpp

This node analyzes the vehicle's timing through different sectors of the track.

- **Inputs**: Latitude, longitude, and velocity.
- **Functionality**:
  - Determines sector transitions using fixed GPS coordinates for each sector.
  - Calculates time spent in each sector and computes average velocity.
  - Sector order is cyclic (1 → 2 → 3 → 1) as the vehicle does not start directly at the first sector.
  - Syncs data based on the slower of the two input frequencies.
- **Outputs**: Publishes a custom message type (`secotor_times.msg`) with sector number, time, and average speed.

## 🖼️ RViz Configuration

Two configuration files are provided in the `rviz/` directory:

- `odom_config.rviz`: Displays the standard TF and odometry topics.
- `odom_config_with_path.rviz`: Includes additional path visualization.

## 🚀 Launch Instructions

Use the provided launch file to start the system:

```bash
roslaunch first_project launch.launch
```

- Launches all three nodes.
- Opens RViz with the basic configuration.
- Initializes parameters (e.g., initial GPS point).
- Includes an optional commented line to switch RViz to the path-enabled version.

## 🧾 Custom Messages

- Custom message defined in `msg/secotor_times.msg`.
- Used to publish structured data on sector performance.

## 🛠️ Build Instructions

Ensure the following ROS dependencies are installed:

- `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `tf`, `std_msgs`

Then build the workspace:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 👨‍💻 Authors

This project was developed by **Francesco Lazzaro** and **Andres Felipe Forero Salas** as part of an academic and practical exploration of vehicle localization using ROS.

---

For further customization, dataset substitution, or circuit changes, feel free to fork and adapt this project.
