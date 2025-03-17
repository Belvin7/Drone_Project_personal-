import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped


class CmdVel(Node):

    def __init__(self):
        super().__init__('cmd_vel')
        self.publisher_ = self.create_publisher(TwistStamped, 'ap/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = 3.
        msg.twist.linear.y = 0.
        msg.twist.linear.z = 0.
        msg.twist.angular.x = 0.
        msg.twist.angular.y = 0.
        msg.twist.angular.z = 0.
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    cmd_vel_publisher = CmdVel()

    rclpy.spin(cmd_vel_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cmd_vel_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
