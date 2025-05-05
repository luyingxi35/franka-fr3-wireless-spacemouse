# Franka Single Arm Controllers and SpaceMouse Publisher

This repository contains ROS 2 packages for controlling a Franka robot arm using a 3Dconnexion SpaceMouse. It includes a `franka_single_arm_controllers` package for Joint Impedance control and a `spacemouse_publisher` package for publishing SpaceMouse input as ROS 2 messages.

## Packages

### 1. `franka_single_arm_controllers`
This package provides a Joint Impedance controller for a single Franka arm. It subscribes to a target cartesian velocity input and sends torque commands to the robot.

#### Key Features:
- Implements a `JointImpedanceController` for controlling the robot's torques.
- Subscribes to `/franka_controller/target_cartesian_velocity_percent` topic for target cartesian velocity input.

#### Launch Files:
- **`franka.launch.py`**: Launches the Franka robot with the controller.
- **`joint_impedance_ik_controller.launch.py`**: Launches the Joint Impedance controller.

### 2. `spacemouse_publisher`
This package provides a ROS 2 node that reads input from a 3Dconnexion SpaceMouse and publishes it as `geometry_msgs/Twist` messages.

#### Key Features:
- Publishes SpaceMouse input to the `/franka_controller/target_cartesian_velocity_percent` topic.
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

## Build and Test

### Building the Project

To build the project, use the following `colcon` command with CMake arguments, required for clang-tidy:

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCHECK_TIDY=ON
```

### Testing of FrankaSingleArmControllers

The FrankaSingleArmControllers package comes with a set of tests, which can be executed using the following command:

```bash
AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=true colcon test --packages-select franka_single_arm_controllers
```

Due to performance issues in cpp-check 2.7, the environment variable AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS must be set to prevent the tests from being skipped.

## Getting Started

To get started with the SpaceMouse publisher and Joint Impedance controller:

1. **Run the SpaceMouse Publisher**  
   Launch the SpaceMouse publisher node to read input from the SpaceMouse and publish it as ROS 2 messages:  
   ```bash
   ros2 launch spacemouse_publisher spacemouse_publisher.launch.py [operator_position_front:=<true_or_false>] [device_path:=<device-path>]
   ```

   The `operator_position_front` parameter is **optional**. If not provided, it defaults to `True`.  
   - Set `operator_position_front:=True` if the operator is sitting in front of the robot.  
   - Set `operator_position_front:=False` if the operator is positioned elsewhere.
   Hint: This aligns the coordinate system of the SpaceMouse to the end-effector. Rotating the SpaceMouse can fulfill the same functionality, if, for example, you are seated on the robot's right side. 

   The `device_path` parameter is **optional**. If not provided, it defaults to `''`, leading to automatic SpaceMouse detection. This parameter is only required, if multiple devices are connected and a specific one shall be used as input device. If not defined, the first one found will be used.


2. **Launch the Joint Impedance Controller**  
   Launch the controller to send torque commands to the Franka robot:  
   ```bash
   ros2 launch franka_single_arm_controllers joint_impedance_ik_controller.launch.py robot_ip:=<robot-ip>
   ```

   Replace `<robot-ip>` with the IP address of your Franka robot.

3. **Launch the Gripper Manager**
   Start the gripper manager node to control the gripper:
   ```bash
   ros2 launch gripper_manager franka_gripper_client.launch.py 
   ```

## Multi-Arm Support

To enable teleoperation with multiple SpaceMice on a single device, you need to specify which SpaceMouse to use for each instance of the SpaceMouse Publisher. This can be achieved using the optional `device_path` parameter, as described in **Getting Started**, section 1.

### Step 1: Identify the Connected SpaceMouse Devices
To determine which connected HID (Human Interface Device) corresponds to each SpaceMouse, run the following command:
```bash
grep -H . /sys/class/hidraw/hidraw*/device/uevent | grep SpaceMouse
```

This will output something like: ` /sys/class/hidraw/hidraw1/device/uevent:HID_NAME=3Dconnexion SpaceMouse Wireless BT` 

In this example, **hidraw1** is the identifier for the SpaceMouse. Based on this, the `device_path` for this SpaceMouse would be: `/dev/hidraw1`

### Step 2: Launch Multiple Instances
Once you have identified the `device_path` for each SpaceMouse, you can launch multiple instances of the SpaceMouse Publisher. To avoid conflicts between instances, use different `ROS_DOMAIN_ID`s for each setup.

Set the `ROS_DOMAIN_ID` environment variable for each terminal before launching the node: 
```bash 
export ROS_DOMAIN_ID=<unique_id>
```

Replace `<unique_id>` with a unique number for each instance. Ensure that all nodes belonging to the same setup use the same `ROS_DOMAIN_ID`.

### Example Workflow
1. Identify the `device_path` for each SpaceMouse (e.g., `/dev/hidraw1`, `/dev/hidraw2`).
2. Open a new terminal for each SpaceMouse instance.
3. Set a unique `ROS_DOMAIN_ID` in each terminal:
```bash
export ROS_DOMAIN_ID=1  # For the first SpaceMouse
export ROS_DOMAIN_ID=2  # For the second SpaceMouse
```
4. Launch the SpaceMouse Publisher for each instance, specifying the corresponding `device_path`:
```bash
ros2 launch spacemouse_publisher spacemouse_publisher.launch.py device_path:=/dev/hidraw1
ros2 launch spacemouse_publisher spacemouse_publisher.launch.py device_path:=/dev/hidraw2
```

By following these steps, you can teleoperate multiple arms or setups simultaneously without conflicts.

### Notes
- The `device_path` parameter is **optional**. If not specified, it defaults to `''`.
- Ensure that each terminal running a SpaceMouse publisher uses a unique `ROS_DOMAIN_ID` to avoid communication conflicts between instances, but remember to set the same `ROS_DOMAIN_ID` for the terminals running the subscribing processes **Gripper Manager** and **Franka Single Arm Controller** controlling the respective robots. 

## Known Issues & Troubleshooting

- **SpaceMouse Not Detected in Dev Container**  
  If the SpaceMouse publisher fails to detect the device while running in a development container, try restarting the container. 

