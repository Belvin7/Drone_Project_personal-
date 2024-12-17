import rclpy
import time 
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
 
class CmdVel(Node):
    def __init__(self):
        super().__init__('cmd_vel')

        # variables
        #self.i = 0
        # TODO: check if we still need send_vel
        self.send_vel = False
        self.msg = TwistStamped()
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.header.frame_id = 'base_link'
        self.msg.twist.linear.x = 0.
        self.msg.twist.linear.y = 0.
        self.msg.twist.linear.z = 0.
        self.msg.twist.angular.x = 0.
        self.msg.twist.angular.y = 0.
        self.msg.twist.angular.z = 0.

        # publisher
        self.cmd_vel_publisher = self.create_publisher(TwistStamped, 'ap/cmd_vel', 10)

        # subscriber
        self.cmd_subscriber = self.create_subscription(String, 'p2/cmd', self.cmd_listener,10)

        # timer with callback
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def cmd_listener(self, msg_sub):
        self.get_logger().info('cmd = %s' % msg_sub.data)
        if msg_sub.data == 'start':
            self.msg.twist.linear.x = 3.0
            self.msg.twist.angular.z = 0.0
            self.send_vel = True
        elif msg_sub.data == 'right':
            self.msg.twist.linear.x = 0.0
            self.msg.twist.angular.z = -1.0
            self.send_vel = True
        elif msg_sub.data == 'left':
            self.msg.twist.linear.x = 0.0
            self.msg.twist.angular.z = 1.0
            self.send_vel = True
        elif msg_sub.data == 'stop':
            self.msg.twist.linear.x = 0.0
            self.msg.twist.angular.z = 0.0
            # this causes problems drone does not stop at once
            #self.send_vel = False 
 
    def timer_callback(self):
        if self.send_vel:
            #msg = TwistStamped()
            self.msg.header.stamp = self.get_clock().now().to_msg()
            #msg.header.frame_id = 'base_link'
            #msg.twist.linear.x = 3.
            #msg.twist.linear.y = 0.
            #msg.twist.linear.z = 0.
            #msg.twist.angular.x = 0.
            #msg.twist.angular.y = 0.
            #msg.twist.angular.z = 0.
            self.cmd_vel_publisher.publish(self.msg)
            #self.get_logger().info('Publishing: "%s"' % msg)
            #self.i += 1


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
