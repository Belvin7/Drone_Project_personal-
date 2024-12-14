import rclpy
import math 
import time 
import errno 
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from builtin_interfaces.msg import Time 
from ardupilot_msgs.msg import GlobalPosition 
from geographic_msgs.msg import GeoPoseStamped 
from geopy import distance 
from geopy import point 
from ardupilot_msgs.srv import ArmMotors 
from ardupilot_msgs.srv import ModeSwitch 
from ardupilot_msgs.srv import Takeoff 
 
COPTER_MODE_GUIDED = 4 
TAKEOFF_ALT = 20.0 

class CmdVel(Node):

    def __init__(self):
        super().__init__('cmd_vel')

        # variables
        self.i = 0
        self.send_vel = False

        # publisher
        self.cmd_vel_publisher = self.create_publisher(TwistStamped, 'ap/cmd_vel', 10)

        # client


        # subscriber
        self.cmd_subscriber = self.create_subscription(String, 'p2/cmd', self.cmd_listener,10)

        # timer with callback
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def cmd_listener(self, msg):
        self.get_logger().info('cmd = %s' % msg.data)
        if msg.data == 'takeoff':
            try:
                if not self.switch_mode_with_timeout(COPTER_MODE_GUIDED, rclpy.duration.Duration(seconds=20)):
                    raise RuntimeError("Unable to switch to guided mode")
                # Block till armed, which will wait for EKF3 to initialize
                if not self.arm_with_timeout(rclpy.duration.Duration(seconds=30)):
                    raise RuntimeError("Unable to arm")

                # Block till in takeoff
                if not self.takeoff_with_timeout(TAKEOFF_ALT, rclpy.duration.Duration(seconds=20)):
                    raise RuntimeError("Unable to takeoff")

                is_ascending_to_takeoff_alt = True
                while is_ascending_to_takeoff_alt:
                    rclpy.spin_once(self)
                    time.sleep(1.0)

                    is_ascending_to_takeoff_alt = self.get_cur_geopose().pose.position.altitude < TAKEOFF_ALT

                if is_ascending_to_takeoff_alt:
                    raise RuntimeError("Failed to reach takeoff altitude")

            except KeyboardInterrupt:
                pass
            self.send_vel = True

    def timer_callback(self):
        if self.send_vel:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x = 3.
            msg.twist.linear.y = 0.
            msg.twist.linear.z = 0.
            msg.twist.angular.x = 0.
            msg.twist.angular.y = 0.
            msg.twist.angular.z = 0.
            self.cmd_vel_publisher.publish(msg)
            #self.get_logger().info('Publishing: "%s"' % msg)
            self.i += 1


def main(args=None):
    rclpy.init(args=args)

    node = CmdVel()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
