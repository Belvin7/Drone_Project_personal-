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
        super().__init__('copter3_cmd_node')
        #ROS parameters
        self.declare_parameter('wanted_distance', 0.)
        self.declare_parameter('v-formation', 0.)
        self.declare_parameter('line-formation', 0.)

        # variables
        self.send_vel = False
        self.send_delay = False
        self.formation = False
        self.v_formation = False
        self.line_formation = False
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

        self.leader_position = np.array((0, 0, 0))
        self.own_position = np.array((4, 0, 0))

        # variables for kalman-filter
        self.state = np.zeros(8)
        self.delta_t = 0.25
        self.Q = np.array([[np.power(self.delta_t,3)/3.0    ,np.power(self.delta_t,2)/2.0  , 0., 0., 0., 0., 0., 0.],
                           [np.power(self.delta_t,2)/2.0    , 1., 0., 0., 0., 0., 0., 0.],
                           [0., 0., np.power(self.delta_t,3)/3.0    ,np.power(self.delta_t,2)/2.0, 0., 0., 0., 0.],
                           [0., 0., np.power(self.delta_t,2)/2.0    , 1., 0., 0., 0., 0.],
                           [0., 0., 0., 0., np.power(self.delta_t,3)/3.0    , np.power(self.delta_t,2)/2.0, 0., 0.],
                           [0., 0., 0., 0., np.power(self.delta_t,2)/2.0 ,1., 0., 0.],
                           [0., 0., 0., 0., 0., 0., np.power(self.delta_t,3)/3.0 , np.power(self.delta_t,2)/2.0],
                           [0., 0., 0., 0., 0., 0., np.power(self.delta_t,2)/2.0 ,1.]])
        self.Cov = np.eye(self.state.size)
        self.A = np.array([[1., 0., 0., 0., self.delta_t, 0., 0., 0.],
                           [0., 1., 0., 0., 0., self.delta_t, 0., 0.],
                           [0., 0., 1., 0., 0., 0., self.delta_t, 0.],
                           [0., 0., 0., 1., 0., 0., 0., self.delta_t],
                           [0., 0., 0., 0., 1., 0., 0., 0.],
                           [0., 0., 0., 0., 0., 1., 0., 0.],
                           [0., 0., 0., 0., 0., 0., 1., 0.],
                           [0., 0., 0., 0., 0., 0., 0., 1.]])
        self.H = np.array([[1., 0., 0., 0., 0., 0., 0., 0.],
                           [0., 1., 0., 0., 0., 0., 0., 0.],
                           [0., 0., 1., 0., 0., 0., 0., 0.],
                           [0., 0., 0., 1., 0., 0., 0., 0.]])
        self.R = 0.01 * np.eye(4)
        self.z = np.zeros(4)

        # publisher
        self.cmd_vel_publisher = self.create_publisher(TwistStamped, '/iris3/setpoint_velocity/cmd_vel', 10)
        
        # subscriber
        self.cmd_subscriber = self.create_subscription(String, 
                                                       'p2/copter3_cmd', 
                                                       self.cmd_listener,
                                                       10)
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

    def lead_listener(self, msg_sub):

        self.leader_position[0] = msg_sub.pose.position.x
        self.leader_position[1] = msg_sub.pose.position.y
        self.leader_position[2] = msg_sub.pose.position.z
        self.z[0] = msg_sub.pose.position.x
        self.z[1] = msg_sub.pose.position.y
        self.z[2] = msg_sub.pose.position.z

        # calc yawn angle from quaternion
        x2 = msg_sub.pose.orientation.x * msg_sub.pose.orientation.x
        z2 = msg_sub.pose.orientation.z * msg_sub.pose.orientation.z

        adbc = msg_sub.pose.orientation.w * msg_sub.pose.orientation.z - msg_sub.pose.orientation.x * msg_sub.pose.orientation.y
        angle = np.arctan2(2. * adbc, 1. - 2. * (z2 + x2))

        if angle >= 0.0 and angle <= np.pi/2.0:
            angle = np.pi/2.0 - angle
        elif angle > np.pi/2.0 and angle < np.pi:
            angle = 2.0*np.pi - angle + np.pi/2.0 
        else:
            angle = np.abs(angle) + np.pi/2.0
        self.z[3] = angle
        self.kalman_predict()
        p = np.random.rand()
        if p >= 0.2:
            self.kalman_update()

    def own_listener(self, msg_sub):
        self.own_position[0] = msg_sub.pose.position.x + 4
        self.own_position[1] = msg_sub.pose.position.y
        self.own_position[2] = msg_sub.pose.position.z

        if self.formation:
            if self.v_formation:
                param_dis = self.get_parameter('wanted_distance').get_parameter_value().double_value
                param_degree = self.get_parameter('v-formation').get_parameter_value().double_value
            elif self.line_formation:
                param_dis = self.get_parameter('wanted_distance').get_parameter_value().double_value
                param_degree = self.get_parameter('line-formation').get_parameter_value().double_value
            theta = np.radians(360) - self.state[3]
            offset = np.array([param_dis * np.cos(np.radians(param_degree)),
                               param_dis * np.sin(np.radians(param_degree)),
                               0])
            rotation_matrix = np.array([
                [np.cos(theta), -np.sin(theta), 0],
                [np.sin(theta), np.cos(theta), 0],
                [0, 0, 1]
            ])

            wanted_pos = self.state[0:3] + rotation_matrix.dot(offset)
            wanted_pos2 = self.leader_position + rotation_matrix.dot(offset)
            kalman_pos_error = np.linalg.norm(wanted_pos - wanted_pos2)
            self.get_logger().info('Kalman Position Error as Distance in Copter 3 = %f' % kalman_pos_error)

            distance_vector = wanted_pos - self.own_position
            #norm_richt = distance_vector / np.linalg.norm(distance_vector)

            self.msg.twist.linear.x = float(distance_vector[0]) * 0.5
            self.msg.twist.linear.y = float(distance_vector[1]) * 0.5
            self.msg.twist.linear.z = float(distance_vector[2]) * 0.5

            self.msg.twist.angular.x = 0.0
            self.msg.twist.angular.y = 0.0
            self.msg.twist.angular.z = self.state[7] * 0.5

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
        elif msg_sub.data == 'v-formation':
            self.formation = True
            self.v_formation = True
            self.line_formation = False
        elif msg_sub.data == 'line-formation':
            self.formation = True
            self.v_formation = False
            self.line_formation = True

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
