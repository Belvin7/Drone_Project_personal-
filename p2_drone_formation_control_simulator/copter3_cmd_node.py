import rclpy
import time 
from rclpy.node import Node

import numpy as np
from std_msgs.msg import String, Float64
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
 
class CmdVel(Node):
    def __init__(self):
        super().__init__('cmd_vel')

        # variables
        self.send_vel = False
        self.send_delay = False
        self.formation = False
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
        self.wanted_distance = 3
        self.degree_offset = 45

        self.leader_position = np.array((0, 0, 0))
        self.formation_orientation = np.array((0, 0, 0, 1))
        self.own_position = np.array((4, 0, 0))
        self.av = np.array((0, 0, 0))
        self.time_ang = self.get_clock().now().nanoseconds * 1e-9
        self.follow_orientation = 0.

        # variables for kalman-filter
        self.state = np.zeros(6)
        self.delta_t = 0.25
        # self.Q = np.eye(self.state.size)
        self.Q = np.array([[np.power(self.delta_t, 3) / 3.0, np.power(self.delta_t, 2) / 2.0, 0., 0., 0., 0.],
                           [np.power(self.delta_t, 2) / 2.0, 1., 0., 0., 0., 0.],
                           [0., 0., np.power(self.delta_t, 3) / 3.0, np.power(self.delta_t, 2) / 2.0, 0., 0.],
                           [0., 0., np.power(self.delta_t, 2) / 2.0, 1., 0., 0.],
                           [0., 0., 0., 0., np.power(self.delta_t, 3) / 3.0, np.power(self.delta_t, 2) / 2.0],
                           [0., 0., 0., 0., np.power(self.delta_t, 2) / 2.0, 1.]])
        self.Cov = np.eye(self.state.size)
        self.A = np.array([[1., 0., 0., self.delta_t, 0., 0.],
                           [0., 1., 0., 0., self.delta_t, 0.],
                           [0., 0., 1., 0., 0., self.delta_t],
                           [0., 0., 0., 1., 0., 0.],
                           [0., 0., 0., 0., 1., 0.],
                           [0., 0., 0., 0., 0., 1.]])
        self.H = np.array([[1., 0., 0., 0., 0., 0.],
                           [0., 1., 0., 0., 0., 0.],
                           [0., 0., 1., 0., 0., 0.]])
        self.R = 0.01 * np.eye(3)
        self.z = np.zeros(3)


        # publisher
        self.cmd_vel_publisher = self.create_publisher(TwistStamped, '/iris3/setpoint_velocity/cmd_vel', 10)
        
        # subscriber
        self.cmd_subscriber = self.create_subscription(String, 
                                                       'p2/copter3_cmd', 
                                                       self.cmd_listener,
                                                       10)
        self.orientation_copter1_subscriber = self.create_subscription(Float64,
                                                                 'iris1/global_position/compass_hdg',
                                                                 self.orientation_copter1_listener,
                                                                 rclpy.qos.qos_profile_sensor_data)
        self.orientation_copter3_subscriber = self.create_subscription(Float64,
                                                                 'iris3/global_position/compass_hdg',
                                                                 self.orientation_copter3_listener,
                                                                 rclpy.qos.qos_profile_sensor_data)
        self.pos_leader_coord_subscriber = self.create_subscription(PoseStamped,
                                                                    'iris1/local_position/pose',
                                                                    self.lead_listener,
                                                                    rclpy.qos.qos_profile_sensor_data)
        self.pos_own = self.create_subscription(PoseStamped,
                                               'iris3/local_position/pose',
                                               self.own_listener,
                                               rclpy.qos.qos_profile_sensor_data)

        # timer with callback
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timer2 = self.create_timer(1.0, self.kalman_timer_callback)

    #based on https://mariogc.com/post/angular-velocity-quaternions/
    def angular_velocities(self, q1, q2, dt):
        return (2 / dt) * np.array([
            q1[3] * q2[0] - q1[0] * q2[3] - q1[1] * q2[2] + q1[2] * q2[1],
            q1[3] * q2[1] + q1[0] * q2[2] - q1[1] * q2[3] - q1[2] * q2[0],
            q1[3] * q2[2] - q1[0] * q2[1] + q1[1] * q2[0] - q1[2] * q2[3]])

    def lead_listener(self, msg_sub):

        self.leader_position[0] = msg_sub.pose.position.x
        self.leader_position[1] = msg_sub.pose.position.y
        self.leader_position[2] = msg_sub.pose.position.z
        self.z[0] = msg_sub.pose.position.x
        self.z[1] = msg_sub.pose.position.y
        self.z[2] = msg_sub.pose.position.z
        self.kalman_predict()
        self.kalman_update()

        new_q = np.array((0, 0, 0, 0))
        new_q[0] = msg_sub.pose.orientation.x
        new_q[1] = msg_sub.pose.orientation.y
        new_q[2] = msg_sub.pose.orientation.z
        new_q[3] = msg_sub.pose.orientation.w

        dtime = self.get_clock().now().nanoseconds * 1e-9 - self.time_ang
        self.av = self.angular_velocities(self.formation_orientation, new_q, dtime)

        self.formation_orientation[0] = msg_sub.pose.orientation.x
        self.formation_orientation[1] = msg_sub.pose.orientation.y
        self.formation_orientation[2] = msg_sub.pose.orientation.z
        self.formation_orientation[3] = msg_sub.pose.orientation.w

        self.time_ang = self.get_clock().now().nanoseconds * 1e-9

    def own_listener(self, msg_sub):
        self.own_position[0] = msg_sub.pose.position.x + 4
        self.own_position[1] = msg_sub.pose.position.y
        self.own_position[2] = msg_sub.pose.position.z

        if self.formation:
            distance = np.linalg.norm(self.own_position - self.leader_position)

            theta = np.radians(self.follow_orientation)
            offset = np.array([self.wanted_distance * np.sin(-np.radians(self.degree_offset)),
                               self.wanted_distance * np.cos(-np.radians(self.degree_offset)),
                               0])
            rotation_matrix = np.array([
                [-np.cos(theta), np.sin(theta), 0],
                [-np.sin(theta), -np.cos(theta), 0],
                [0, 0, 1]
            ])

            wanted_pos = self.leader_position + rotation_matrix.dot(offset)

            richtungsvektor = wanted_pos - self.own_position
            norm_richt = richtungsvektor / np.linalg.norm(richtungsvektor)

            self.get_logger().info('Distanz Copter 3= %f ' % distance)
            #self.get_logger().info('Copter 3 position: %f %f %f' % wanted_pos[0] % wanted_pos[1] % wanted_pos[2])
            self.msg.twist.linear.x = float(richtungsvektor[0]) * 0.5
            self.msg.twist.linear.y = float(richtungsvektor[1]) * 0.5
            self.msg.twist.linear.z = float(richtungsvektor[2]) * 0.5

            self.msg.twist.angular.x = self.av[0]
            self.msg.twist.angular.y = self.av[1]
            self.msg.twist.angular.z = self.av[2]
            self.send_vel = True
            self.send_delay = True

    def cmd_listener(self, msg_sub):
        #self.get_logger().info('cmd = %s' % msg_sub.data)
        if msg_sub.data == 'start':
            self.msg.twist.linear.x  = np.sin(2*np.pi/360 * float(self.orientation)) * 3.0 
            self.msg.twist.linear.y  = np.cos(2*np.pi/360 * float(self.orientation)) * 3.0
            self.msg.twist.angular.z = 0.0
            self.send_vel = True
            self.send_delay = True
            self.formation = False
        elif msg_sub.data == 'right':
            self.msg.twist.linear.x  = 0.0
            self.msg.twist.linear.y  = 0.0
            self.msg.twist.angular.z = -1.0
            self.send_vel = True
            self.send_delay = True
            self.formation = False
        elif msg_sub.data == 'left':
            self.msg.twist.linear.x  = 0.0
            self.msg.twist.linear.y  = 0.0
            self.msg.twist.angular.z = 1.0
            self.send_vel = True
            self.send_delay = True
            self.formation = False
        elif msg_sub.data == 'stop':
            self.msg.twist.linear.x  = 0.0
            self.msg.twist.linear.y  = 0.0
            self.msg.twist.angular.z = 0.0
            self.send_vel = False
            self.formation = False
        elif msg_sub.data == 'formation':
            self.formation = True

    def orientation_copter1_listener(self, msg_sub):
        self.follow_orientation = msg_sub.data

    def orientation_copter3_listener(self, msg_sub):
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

    def kalman_timer_callback(self):
        self.get_logger().info('-copter3-------------------------------------')
        self.get_logger().info('self.state[0] = %f ' % self.state[0])
        self.get_logger().info('self.state[1] = %f ' % self.state[1])
        self.get_logger().info('self.state[2] = %f ' % self.state[2])
        self.get_logger().info('self.state[3] = %f ' % self.state[3])
        self.get_logger().info('self.state[4] = %f ' % self.state[4])
        self.get_logger().info('self.state[5] = %f ' % self.state[5])

    def kalman_predict(self):
        self.state = self.A @ self.state
        self.Cov = self.A @ self.Cov @ self.A.transpose() + self.Q

    def kalman_update(self):
        S = self.H @ self.Cov @ self.H.transpose() + self.R
        K = self.Cov @ self.H.transpose() @ np.linalg.inv(S)
        self.state = self.state + K @ (self.z - self.H @ self.state)
        #self.Cov = self.Cov - K @ self.H @ self.Cov
        self.Cov = self.Cov - K @ S @ K.transpose()


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
