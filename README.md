# Franka Single Arm Controllers and SpaceMouse Publisher

This repository contains ROS 2 packages for controlling a Franka robot arm using a 3Dconnexion SpaceMouse. It includes a `franka_single_arm_controllers` package for Cartesian velocity control and a `spacemouse_publisher` package for publishing SpaceMouse input as ROS 2 messages.

## Packages

### 1. `franka_single_arm_controllers`
This package provides a Cartesian velocity controller for a single Franka arm. It subscribes to a target cartesian velocity input and sends velocity commands to the robot.

#### Key Features:
- Implements a `CartesianVelocityController` for controlling the robot's Cartesian velocity.
- Subscribes to `/franka_controller/target_cartesian_velocity` topic for target cartesian velocity input.
- Configurable parameters for velocity limits and collision behavior.

#### Launch Files:
- **`franka.launch.py`**: Launches the Franka robot with the controller.
- **`cartesian_velocity_controller.launch.py`**: Launches the Cartesian velocity controller.

### 2. `spacemouse_publisher`
This package provides a ROS 2 node that reads input from a 3Dconnexion SpaceMouse and publishes it as `geometry_msgs/Twist` messages.

#### Key Features:
- Publishes SpaceMouse input to the `/franka_controller/target_cartesian_velocity` topic.
- Uses the `pyspacemouse` library for device communication.

#### Launch Files:
- **`spacemouse_publisher.launch.py`**: Launches the SpaceMouse publisher node.

### 3. `gripper_manager`
This package provides a ROS 2 node for managing the gripper of the Franka robot. It allows sending commands to control the gripper's width and perform homing actions.

#### Key Features:
- Subscribes to `/gripper_client/target_gripper_width_percent` for gripper width commands.
- Supports homing and move actions for the gripper.

#### Launch Files:
- **`franka_gripper_client.launch.py`**: Launches the gripper manager node.

## Getting Started

To get started with the SpaceMouse publisher and Cartesian velocity controller:

1. **Run the SpaceMouse Publisher**  
   Launch the SpaceMouse publisher node to read input from the SpaceMouse and publish it as ROS 2 messages:  
   ```bash
   ros2 launch spacemouse_publisher spacemouse_publisher.launch.py [operator_position_front:=<true_or_false>]
   ```

   The `operator_position_front` parameter is **optional**. If not provided, it defaults to `True`.  
   - Set `operator_position_front:=True` if the operator is sitting in front of the robot.  
   - Set `operator_position_front:=False` if the operator is positioned elsewhere.
   Hint: This aligns the coordinate system of the SpaceMouse to the end-effector. Rotating the SpaceMouse can fulfill the same functionality, if, for example, you are seated on the robot's right side. 


2. **Launch the Cartesian Velocity Controller**  
   Launch the controller to send velocity commands to the Franka robot:  
   ```bash
   ros2 launch franka_single_arm_controllers cartesian_velocity_controller.launch.py robot_ip:=<robot-ip>
   ```

   Replace `<robot-ip>` with the IP address of your Franka robot.

3. **Launch the Gripper Manager**
   Start the gripper manager node to control the gripper:
   ```bash
   ros2 launch gripper_manager franka_gripper_client.launch.py 
   ```

## Known Issues & Troubleshooting

- **SpaceMouse Not Detected in Dev Container**  
  If the SpaceMouse publisher fails to detect the device while running in a development container, try restarting the container. 

