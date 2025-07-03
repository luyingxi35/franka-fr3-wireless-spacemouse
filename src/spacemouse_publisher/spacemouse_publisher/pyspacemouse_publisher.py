import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import pyspacemouse
from sensor_msgs.msg import Image
from franka_msgs.msg import FrankaRobotState
import os, csv


class SpaceMousePublisher(Node):
    """
    A ROS2 Node that publishes 3D mouse input as Twist messages.

    This class initializes a ROS2 publisher to publish geometry_msgs/Twist messages
    based on the input from a 3D SpaceMouse device. It uses the pyspacemouse library
    to read the device state and publishes the corresponding linear and angular
    velocities at a fixed rate.
    """
    
    def __init__(self):
        super().__init__('spacemouse_publisher')
        self.get_logger().info("Initializing SpaceMouse publisher...")

        self.declare_parameter('operator_position_front', True)
        self._operator_position_front = self.get_parameter('operator_position_front').get_parameter_value().bool_value
        self.get_logger().info(f"Operator position front: {self._operator_position_front}")

        self.declare_parameter('device_path', '')
        self._device_path = self.get_parameter('device_path').get_parameter_value().string_value

        self._twist_publisher = self.create_publisher(Twist, '/franka_controller/target_cartesian_velocity_percent', 10)
        self._gripper_width_publisher = self.create_publisher(Float32, '/gripper_client/target_gripper_width_percent', 10)
        self._timer = self.create_timer(0.01, self._timer_callback) 
        self._device_open_success = pyspacemouse.open(dof_callback=None, button_callback_arr=[
            pyspacemouse.ButtonCallback(0, self._button_callback),  # Button 1
            pyspacemouse.ButtonCallback(14, self._button_callback)   # Button 2
        ], path=self._device_path)

        # Setup logger
        self._state_robot = None
        # self._rgb = None

        self.create_subscription(FrankaRobotState,'/franka_robot_state_broadcaster/robot_state', self._state_cb, 10)
        # self.create_subscription(Image,'/camera/color/image_raw', self._image_cb, 10)

        # Set logger file
        os.makedirs('teleop_logs', exist_ok=True)
        fpath = os.path.join('teleop_logs', 'teleop_log.csv')
        self._csv = open(fpath, 'w', newline='')
        self._writer = csv.writer(self._csv)
        self._writer.writerow([
            # 'timestamp',
            'twist_lin_x','twist_lin_y','twist_lin_z',
            'twist_ang_x','twist_ang_y','twist_ang_z',
            'joint_positions',
            # 'image_stamp'
        ])

    # State reading function
    def _state_cb(self, msg):
        self._state_robot = msg

    def _no_action(self, twist_msg):
        if twist_msg.linear.x == 0.0 and twist_msg.linear.y == 0.0 and twist_msg.linear.z == 0.0 and \
           twist_msg.angular.x == 0.0 and twist_msg.angular.y == 0.0 and twist_msg.angular.z == 0.0:
            return True
        return False

    def _timer_callback(self):
        if not self._device_open_success:
            return

        state = pyspacemouse.read()

        # print(state)
        
        twist_msg = Twist()
        twist_msg.linear.x = -float(state.y)
        twist_msg.linear.y = float(state.x)
        twist_msg.linear.z = float(state.z)
        twist_msg.angular.x = -float(state.roll)
        twist_msg.angular.y = -float(state.pitch)
        twist_msg.angular.z = -float(state.yaw)

        if not self._operator_position_front:
            twist_msg.linear.x *= -1
            twist_msg.linear.y *= -1
            twist_msg.angular.x *= -1
            twist_msg.angular.y *= -1

        # Log the data
        # if self._state_robot and self._rgb:
        if self._state_robot and not self._no_action(twist_msg):
            # t = self._rgb.header.stamp.sec + self._rgb.header.stamp.nanosec*1e-9
            # t = self._rgb.header.stamp.sec 
            joints = list(self._state_robot.measured_joint_state.position)
            row = [
                # f"{t:.6f}",
                twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z,
                twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z,
                joints,
                # f"{self._rgb.header.stamp.sec}.{self._rgb.header.stamp.nanosec:09d}"
            ]
            self._writer.writerow(row)
            # self.get_logger().debug(f"Logged @ {t:.3f}")

        self._twist_publisher.publish(twist_msg)

    def _button_callback(self, state, buttons, pressed_buttons):
        self.get_logger().info(str(pressed_buttons))
        target_gripper_width_percent_msg = Float32()
        if 0 == pressed_buttons:
            # print("Button 1 pressed")
            self.get_logger().info("Button 1 pressed")
            target_gripper_width_percent_msg.data = 0.0
            
        if 14 == pressed_buttons:
            # print("Button 2 pressed")
            self.get_logger().info("Button 2 pressed")
            target_gripper_width_percent_msg.data = 1.0
            
        self._gripper_width_publisher.publish(target_gripper_width_percent_msg)

    def destroy_node(self):
        self._csv.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    spacemouse_publisher = SpaceMousePublisher()
    rclpy.spin(spacemouse_publisher)
    spacemouse_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()