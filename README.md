# UR RTDE ROS Package

This ROS package provides an interface to control a Universal Robots (UR) robot using the Real-Time Data Exchange (RTDE) protocol. It includes services for setting modes, moving to poses, scheduling waypoints, and retrieving TCP and joint states.

## Features

- Connect to a UR robot using RTDE.
- Control the robot using various modes (IDLE, MOVEL, SERVOL, STOP).
- Move the robot to specified poses.
- Schedule waypoints for the robot to follow.
- Retrieve the current TCP and joint states of the robot.

## Dependencies
- `numpy`
- `rospy`
- `rtde_control` (RTDE Control Interface)
- `rtde_receive` (RTDE Receive Interface)
- [`ur_rtde_ros`](command:_github.copilot.openSymbolFromReferences?%5B%7B%22%24mid%22%3A1%2C%22path%22%3A%22%2Fhome%2Fhaoran_zheng%2FDocuments%2FLfD_tactile%2FAvatar-ROS%2Fdevel%2Flib%2Fpython3%2Fdist-packages%2Fur_rtde_ros%2F__init__.py%22%2C%22scheme%22%3A%22file%22%7D%2C%7B%22line%22%3A0%2C%22character%22%3A0%7D%5D "devel/lib/python3/dist-packages/ur_rtde_ros/__init__.py") (Custom ROS services)
- [`utils.pose_trajectory_interpolator`](command:_github.copilot.openSymbolFromReferences?%5B%7B%22%24mid%22%3A1%2C%22path%22%3A%22%2Fhome%2Fhaoran_zheng%2FDocuments%2FLfD_tactile%2FAvatar-ROS%2Fsrc%2Fur_rtde_ros%2Futils%2F__init__.py%22%2C%22scheme%22%3A%22file%22%7D%2C%7B%22line%22%3A0%2C%22character%22%3A0%7D%5D "src/ur_rtde_ros/utils/__init__.py") (Pose Trajectory Interpolator)

## Installation

1. Clone the repository into your ROS workspace:
    ```sh
    cd ~/catkin_ws/src
    git clone <repository_url>
    ```

2. Install the required dependencies:
    ```sh
    pip install numpy
    ```

3. Build the workspace:
    ```sh
    cd ~/catkin_ws
    catkin_make
    ```

4. Source the workspace:
    ```sh
    source devel/setup.bash
    ```

## Usage

1. Launch the ROS node:
    ```sh
    rosrun ur_rtde_ros ur_node.py
    ```

2. Use the provided services to control the robot.

### Services

- **set_mode**: Set the mode of the robot.
    ```sh
    rosservice call /set_mode "mode: 'MOVEL'"
    ```

- **moveL**: Move the robot to a specified pose.
    ```sh
    rosservice call /moveL "pose: [0.5, 0.0, 0.5, 0.0, 1.57, 0.0] vel: 0.5 acc: 0.5"
    ```

- **schedule_waypoint**: Schedule a waypoint for the robot.
    ```sh
    rosservice call /schedule_waypoint "pose: [0.5, 0.0, 0.5, 0.0, 1.57, 0.0] target_time: 5.0"
    ```

- **get_tcp_states**: Get the current TCP pose and speed.
    ```sh
    rosservice call /get_tcp_states "{}"
    ```

- **get_joint_states**: Get the current joint positions and speeds.
    ```sh
    rosservice call /get_joint_states "{}"
    ```

## Node Details

### [`URNode`](command:_github.copilot.openSymbolFromReferences?%5B%7B%22%24mid%22%3A1%2C%22path%22%3A%22%2Fhome%2Fhaoran_zheng%2FDocuments%2FLfD_tactile%2FAvatar-ROS%2Fsrc%2Fur_rtde_ros%2Fur_node.py%22%2C%22scheme%22%3A%22file%22%7D%2C%7B%22line%22%3A23%2C%22character%22%3A6%7D%5D "src/ur_rtde_ros/ur_node.py")

The main class that handles the connection to the UR robot and provides the services.

#### Methods

- [`__init__`](command:_github.copilot.openSymbolFromReferences?%5B%7B%22%24mid%22%3A1%2C%22path%22%3A%22%2Fhome%2Fhaoran_zheng%2FDocuments%2FLfD_tactile%2FAvatar-ROS%2Fsrc%2Fur_rtde_ros%2Futils%2Fpose_trajectory_interpolator.py%22%2C%22scheme%22%3A%22file%22%7D%2C%7B%22line%22%3A24%2C%22character%22%3A8%7D%5D "src/ur_rtde_ros/utils/pose_trajectory_interpolator.py"): Initializes the node, connects to the robot, and sets up services.
- [`close`](command:_github.copilot.openSymbolFromReferences?%5B%7B%22%24mid%22%3A1%2C%22path%22%3A%22%2Fhome%2Fhaoran_zheng%2FDocuments%2FLfD_tactile%2FAvatar-ROS%2Fsrc%2Fur_rtde_ros%2Fur_node.py%22%2C%22scheme%22%3A%22file%22%7D%2C%7B%22line%22%3A53%2C%22character%22%3A8%7D%5D "src/ur_rtde_ros/ur_node.py"): Stops the robot and disconnects.
- [`set_mode`](command:_github.copilot.openSymbolFromReferences?%5B%7B%22%24mid%22%3A1%2C%22path%22%3A%22%2Fhome%2Fhaoran_zheng%2FDocuments%2FLfD_tactile%2FAvatar-ROS%2Fdevel%2Flib%2Fpython3%2Fdist-packages%2Fur_rtde_ros%2Fsrv%2F_set_mode.py%22%2C%22scheme%22%3A%22file%22%7D%2C%7B%22line%22%3A242%2C%22character%22%3A6%7D%5D "devel/lib/python3/dist-packages/ur_rtde_ros/srv/_set_mode.py"): Sets the mode of the robot.
- [`get_tcp_states`](command:_github.copilot.openSymbolFromReferences?%5B%7B%22%24mid%22%3A1%2C%22path%22%3A%22%2Fhome%2Fhaoran_zheng%2FDocuments%2FLfD_tactile%2FAvatar-ROS%2Fsrc%2Fur_rtde_ros%2Fur_node.py%22%2C%22scheme%22%3A%22file%22%7D%2C%7B%22line%22%3A66%2C%22character%22%3A8%7D%5D "src/ur_rtde_ros/ur_node.py"): Retrieves the current TCP pose and speed.
- [`get_joint_states`](command:_github.copilot.openSymbolFromReferences?%5B%7B%22%24mid%22%3A1%2C%22path%22%3A%22%2Fhome%2Fhaoran_zheng%2FDocuments%2FLfD_tactile%2FAvatar-ROS%2Fsrc%2Fur_rtde_ros%2Fur_node.py%22%2C%22scheme%22%3A%22file%22%7D%2C%7B%22line%22%3A71%2C%22character%22%3A8%7D%5D "src/ur_rtde_ros/ur_node.py"): Retrieves the current joint positions and speeds.
- [`move_to_pose`](command:_github.copilot.openSymbolFromReferences?%5B%7B%22%24mid%22%3A1%2C%22path%22%3A%22%2Fhome%2Fhaoran_zheng%2FDocuments%2FLfD_tactile%2FAvatar-ROS%2Fsrc%2Fur_rtde_ros%2Fur_node.py%22%2C%22scheme%22%3A%22file%22%7D%2C%7B%22line%22%3A76%2C%22character%22%3A8%7D%5D "src/ur_rtde_ros/ur_node.py"): Moves the robot to a specified pose.
- [`schedule_waypoint`](command:_github.copilot.openSymbolFromReferences?%5B%7B%22%24mid%22%3A1%2C%22path%22%3A%22%2Fhome%2Fhaoran_zheng%2FDocuments%2FLfD_tactile%2FAvatar-ROS%2Fsrc%2Fur_rtde_ros%2Futils%2Fpose_trajectory_interpolator.py%22%2C%22scheme%22%3A%22file%22%7D%2C%7B%22line%22%3A123%2C%22character%22%3A8%7D%5D "src/ur_rtde_ros/utils/pose_trajectory_interpolator.py"): Schedules a waypoint for the robot.
- [`servo_loop`](command:_github.copilot.openSymbolFromReferences?%5B%7B%22%24mid%22%3A1%2C%22path%22%3A%22%2Fhome%2Fhaoran_zheng%2FDocuments%2FLfD_tactile%2FAvatar-ROS%2Fsrc%2Fur_rtde_ros%2Fur_node.py%22%2C%22scheme%22%3A%22file%22%7D%2C%7B%22line%22%3A106%2C%22character%22%3A8%7D%5D "src/ur_rtde_ros/ur_node.py"): Main loop for the servo mode.

### [`PoseTrajectoryInterpolator`](command:_github.copilot.openSymbolFromReferences?%5B%7B%22%24mid%22%3A1%2C%22path%22%3A%22%2Fhome%2Fhaoran_zheng%2FDocuments%2FLfD_tactile%2FAvatar-ROS%2Fsrc%2Fur_rtde_ros%2Futils%2Fpose_trajectory_interpolator.py%22%2C%22scheme%22%3A%22file%22%7D%2C%7B%22line%22%3A23%2C%22character%22%3A6%7D%5D "src/ur_rtde_ros/utils/pose_trajectory_interpolator.py")

A utility class for interpolating poses over time.

#### Methods

- [`__init__`](command:_github.copilot.openSymbolFromReferences?%5B%7B%22%24mid%22%3A1%2C%22path%22%3A%22%2Fhome%2Fhaoran_zheng%2FDocuments%2FLfD_tactile%2FAvatar-ROS%2Fsrc%2Fur_rtde_ros%2Futils%2Fpose_trajectory_interpolator.py%22%2C%22scheme%22%3A%22file%22%7D%2C%7B%22line%22%3A24%2C%22character%22%3A8%7D%5D "src/ur_rtde_ros/utils/pose_trajectory_interpolator.py"): Initializes the interpolator with times and poses.
- [`times`](command:_github.copilot.openSymbolFromReferences?%5B%7B%22%24mid%22%3A1%2C%22path%22%3A%22%2Fhome%2Fhaoran_zheng%2FDocuments%2FLfD_tactile%2FAvatar-ROS%2Fsrc%2Fur_rtde_ros%2Futils%2Fpose_trajectory_interpolator.py%22%2C%22scheme%22%3A%22file%22%7D%2C%7B%22line%22%3A24%2C%22character%22%3A23%7D%5D "src/ur_rtde_ros/utils/pose_trajectory_interpolator.py"): Returns the times of the interpolator.
- [`poses`](command:_github.copilot.openSymbolFromReferences?%5B%7B%22%24mid%22%3A1%2C%22path%22%3A%22%2Fhome%2Fhaoran_zheng%2FDocuments%2FLfD_tactile%2FAvatar-ROS%2Fsrc%2Fur_rtde_ros%2Futils%2Fpose_trajectory_interpolator.py%22%2C%22scheme%22%3A%22file%22%7D%2C%7B%22line%22%3A24%2C%22character%22%3A42%7D%5D "src/ur_rtde_ros/utils/pose_trajectory_interpolator.py"): Returns the poses of the interpolator.
- [`trim`](command:_github.copilot.openSymbolFromReferences?%5B%7B%22%24mid%22%3A1%2C%22path%22%3A%22%2Fhome%2Fhaoran_zheng%2FDocuments%2FLfD_tactile%2FAvatar-ROS%2Fsrc%2Fur_rtde_ros%2Futils%2Fpose_trajectory_interpolator.py%22%2C%22scheme%22%3A%22file%22%7D%2C%7B%22line%22%3A65%2C%22character%22%3A8%7D%5D "src/ur_rtde_ros/utils/pose_trajectory_interpolator.py"): Trims the interpolator to a specified time range.
- [`drive_to_waypoint`](command:_github.copilot.openSymbolFromReferences?%5B%7B%22%24mid%22%3A1%2C%22path%22%3A%22%2Fhome%2Fhaoran_zheng%2FDocuments%2FLfD_tactile%2FAvatar-ROS%2Fsrc%2Fur_rtde_ros%2Futils%2Fpose_trajectory_interpolator.py%22%2C%22scheme%22%3A%22file%22%7D%2C%7B%22line%22%3A86%2C%22character%22%3A8%7D%5D "src/ur_rtde_ros/utils/pose_trajectory_interpolator.py"): Drives the current pose to a specified waypoint.

## License

This project is licensed under the MIT License. See the LICENSE file for details.

## Acknowledgements

This package uses the RTDE Control and Receive interfaces provided by Universal Robots. Special thanks to the ROS and UR communities for their support and contributions.

For more information, please refer to the official documentation and user guides provided by Universal Robots.