import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pyspacemouse


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
        self.publisher_ = self.create_publisher(Twist, 'franka_controller/target_cartesian_velocity', 10)
        self.timer = self.create_timer(0.01, self.timer_callback) 
        self.success = pyspacemouse.open()

    def timer_callback(self):
        if self.success:
            state = pyspacemouse.read()
            twist_msg = Twist()
            twist_msg.linear.x = float(state.x)
            twist_msg.linear.y = float(state.y)
            twist_msg.linear.z = float(state.z)
            twist_msg.angular.x = float(state.roll)
            twist_msg.angular.y = float(state.pitch)
            twist_msg.angular.z = float(state.yaw)
            self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    spacemouse_publisher = SpaceMousePublisher()
    rclpy.spin(spacemouse_publisher)
    spacemouse_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()