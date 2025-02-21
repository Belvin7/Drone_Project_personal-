import rclpy
import time 
from rclpy.node import Node

import numpy as np
from std_msgs.msg import String, Float64
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import NavSatFix
 
class CmdVel(Node):
    def __init__(self):
        super().__init__('cmd_vel')

        # variables
        self.send_vel = False
        self.send_delay = False
        self.msg = TwistStamped()
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.header.frame_id = 'base_link'
        self.msg.twist.linear.x = 0.
        self.msg.twist.linear.y = 0.
        self.msg.twist.linear.z = 0.
        self.msg.twist.angular.x = 0.
        self.msg.twist.angular.y = 0.
        self.msg.twist.angular.z = 0.
        self.orientation = 0.

        # variables for kalman-filter
        self.follow_lat = 0.
        self.follow_log = 0.
        self.follow_alt = 0.
        self.follow_cov = np.zeros(9)
        self.follow_orientation = 0.

        # publisher
        #self.cmd_vel_publisher = self.create_publisher(TwistStamped, '/iris1/setpoint_attitude/cmd_vel', 10)
        self.cmd_vel_publisher = self.create_publisher(TwistStamped, '/iris1/setpoint_velocity/cmd_vel', 10)
        
        # subscriber
        self.cmd_subscriber = self.create_subscription(String, 
                                                       'p2/copter1_cmd', 
                                                       self.cmd_listener, 
                                                       10)
        self.pos_copter2_subscriber = self.create_subscription(NavSatFix,
                                                               'iris2/global_position/global',
                                                               self.pos_copter2_listener,
                                                               rclpy.qos.qos_profile_sensor_data)
        self.orientation_copter2_subscriber = self.create_subscription(Float64,
                                                                 'iris2/global_position/compass_hdg',
                                                                 self.orientation_copter2_listener,
                                                                 rclpy.qos.qos_profile_sensor_data)
        self.orientation_copter1_subscriber = self.create_subscription(Float64,
                                                                 'iris1/global_position/compass_hdg',
                                                                 self.orientation_copter1_listener,
                                                                 rclpy.qos.qos_profile_sensor_data)


        # timer with callback
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def cmd_listener(self, msg_sub):
        #self.get_logger().info('cmd = %s' % msg_sub.data)
        if msg_sub.data == 'start':
            self.msg.twist.linear.x  = np.sin(2*np.pi/360 * float(self.orientation)) * 3.0 
            self.msg.twist.linear.y  = np.cos(2*np.pi/360 * float(self.orientation)) * 3.0
            self.msg.twist.angular.z = 0.0
            self.send_vel = True
            self.send_delay = True
        elif msg_sub.data == 'right':
            self.msg.twist.linear.x  = 0.0
            self.msg.twist.linear.y  = 0.0
            self.msg.twist.angular.z = -1.0
            self.send_vel = True
            self.send_delay = True
        elif msg_sub.data == 'left':
            self.msg.twist.linear.x  = 0.0
            self.msg.twist.linear.y  = 0.0
            self.msg.twist.angular.z = 1.0
            self.send_vel = True
            self.send_delay = True
        elif msg_sub.data == 'stop':
            self.msg.twist.linear.x  = 0.0
            self.msg.twist.linear.y  = 0.0
            self.msg.twist.angular.z = 0.0
            self.send_vel = False 

    def pos_copter2_listener(self, msg_sub):
        # insert for debug
        #if self.send_vel:
        #    self.get_logger().info('copter1: copter2.latitude  = %f' % msg_sub.latitude)
        #    self.get_logger().info('copter1: copter2.longitude = %f' % msg_sub.longitude)
        #    self.get_logger().info('copter1: copter2.altitude  = %f' % msg_sub.altitude)
        self.follow_lat = msg_sub.latitude
        self.follow_log = msg_sub.longitude
        self.follow_alt = msg_sub.altitude
        self.follow_cov = msg_sub.position_covariance

    def orientation_copter2_listener(self, msg_sub):
        self.follow_orientation = msg_sub

    def orientation_copter1_listener(self, msg_sub):
        self.orientation = msg_sub.data

    def timer_callback(self):
        if self.send_vel and self.send_delay:
            self.msg.header.stamp = self.get_clock().now().to_msg()
            self.cmd_vel_publisher.publish(self.msg)
            #self.get_logger().info('Publishing: "%s"' % self.msg)
        elif not self.send_vel and self.send_delay:
            self.msg.header.stamp = self.get_clock().now().to_msg()
            self.cmd_vel_publisher.publish(self.msg)
            #self.get_logger().info('Publishing: "%s"' % self.msg)
            self.send_delay = False

 
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
