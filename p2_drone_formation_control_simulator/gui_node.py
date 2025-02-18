import rclpy
import time
import tkinter as tk
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL

#COPTER_MODE_GUIDED = 4 
#TAKEOFF_ALT = 20.0 

class P2(Node):
    def __init__(self):
        super().__init__('p2')

        # publisher should be used to set the velocities
        self.copter1_cmd_publisher = self.create_publisher(String, 'p2/copter1_cmd', 10)
        self.copter2_cmd_publisher = self.create_publisher(String, 'p2/copter2_cmd', 10)
        self.copter3_cmd_publisher = self.create_publisher(String, 'p2/copter3_cmd', 10)

        # clients for copter 1
        self.declare_parameter("copter1_mode_topic", "/iris1/set_mode")
        self.copter1_mode_topic = self.get_parameter("copter1_mode_topic").get_parameter_value().string_value
        self.copter1_client_mode_switch = self.create_client(SetMode, self.copter1_mode_topic)
        while not self.copter1_client_mode_switch.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('copter1 mode switch service not available, waiting again...')

        self.declare_parameter("copter1_arm_topic", "/iris1/cmd/arming")
        self.copter1_arm_topic = self.get_parameter("copter1_arm_topic").get_parameter_value().string_value
        self.copter1_client_arm = self.create_client(CommandBool, self.copter1_arm_topic)
        while not self.copter1_client_arm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('copter1 arm service not available, waiting again...')

        self.declare_parameter("copter1_takeoff_service", "/iris1/cmd/takeoff")
        self.copter1_takeoff_topic = self.get_parameter("copter1_takeoff_service").get_parameter_value().string_value
        self.copter1_client_takeoff = self.create_client(CommandTOL, self.copter1_takeoff_topic)
        while not self.copter1_client_takeoff.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('copter1 takeoff service not available, waiting again...')

        # clients for copter 2
        self.declare_parameter("copter2_mode_topic", "/iris2/set_mode")
        self.copter2_mode_topic = self.get_parameter("copter2_mode_topic").get_parameter_value().string_value
        self.copter2_client_mode_switch = self.create_client(SetMode, self.copter2_mode_topic)
        while not self.copter2_client_mode_switch.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('copter2 mode switch service not available, waiting again...')

        self.declare_parameter("copter2_arm_topic", "/iris2/cmd/arming")
        self.copter2_arm_topic = self.get_parameter("copter2_arm_topic").get_parameter_value().string_value
        self.copter2_client_arm = self.create_client(CommandBool, self.copter2_arm_topic)
        while not self.copter2_client_arm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('copter2 arm service not available, waiting again...')

        self.declare_parameter("copter2_takeoff_service", "/iris2/cmd/takeoff")
        self.copter2_takeoff_topic = self.get_parameter("copter2_takeoff_service").get_parameter_value().string_value
        self.copter2_client_takeoff = self.create_client(CommandTOL, self.copter2_takeoff_topic)
        while not self.copter2_client_takeoff.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('copter2 takeoff service not available, waiting again...')

        # clients for copter 3
        self.declare_parameter("copter3_mode_topic", "/iris3/set_mode")
        self.copter3_mode_topic = self.get_parameter("copter3_mode_topic").get_parameter_value().string_value
        self.copter3_client_mode_switch = self.create_client(SetMode, self.copter3_mode_topic)
        while not self.copter3_client_mode_switch.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('copter3 mode switch service not available, waiting again...')

        self.declare_parameter("copter3_arm_topic", "/iris3/cmd/arming")
        self.copter3_arm_topic = self.get_parameter("copter3_arm_topic").get_parameter_value().string_value
        self.copter3_client_arm = self.create_client(CommandBool, self.copter3_arm_topic)
        while not self.copter3_client_arm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('copter3 arm service not available, waiting again...')

        self.declare_parameter("copter3_takeoff_service", "/iris3/cmd/takeoff")
        self.copter3_takeoff_topic = self.get_parameter("copter3_takeoff_service").get_parameter_value().string_value
        self.copter3_client_takeoff = self.create_client(CommandTOL, self.copter3_takeoff_topic)
        while not self.copter3_client_takeoff.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('copter3 takeoff service not available, waiting again...')


    def gui(self):
        self.window = tk.Tk()
        self.window.title('P2 Simulator')

        l1 = tk.Label(self.window, text = "Copter 1")
        l2 = tk.Label(self.window, text = "Copter 2")
        l3 = tk.Label(self.window, text = "Copter 3")
        l1.grid(column=0, row=0, pady=0)
        l2.grid(column=1, row=0, pady=0)
        l3.grid(column=2, row=0, pady=0)

        ## first drone
        # guided button
        guided1_button = tk.Button(self.window,
          text='Guided-mode', 
          command=self.guided1_clicked,
          padx=10,
          pady=5,
          width=15
        )
        guided1_button.grid(column=0, row=1, padx=10, pady=0)

        # arm button
        arm1_button = tk.Button(self.window,
          text='Arm', 
          command=self.arm1_clicked,
          padx=10,
          pady=5,
          width=15
        )
        arm1_button.grid(column=0, row=2, padx=10, pady=0)

        # create takeoff button
        takeoff1_button = tk.Button(self.window, 
          text='Takeoff', 
          command=self.takeoff1_clicked,
          padx=10,
          pady=5,
          width=15
        )
        takeoff1_button.grid(column=0, row=3, padx=10, pady=0)

        # start moving button
        start1_button = tk.Button(self.window, 
          text='Start', 
          command=self.start1_clicked,
          padx=10,
          pady=5,
          width=15
        )
        start1_button.grid(column=0, row=4, padx=10, pady=0)

        # left turning button
        left1_button = tk.Button(self.window, 
          text='Left-turning', 
          command=self.left1_clicked,
          padx=10,
          pady=5,
          width=15
        )
        left1_button.grid(column=0, row=5, padx=10, pady=0)

        # right turning button
        right1_button = tk.Button(self.window, 
          text='Right-turning', 
          command=self.right1_clicked,
          padx=10,
          pady=5,
          width=15
        )
        right1_button.grid(column=0, row=6, padx=10, pady=0)

        # stop moving button
        stop1_button = tk.Button(self.window, 
          text='Stop', 
          command=self.stop1_clicked,
          padx=10,
          pady=5,
          width=15
        )
        stop1_button.grid(column=0, row=7, padx=10, pady=0)

        # return-to-launch button
        rtl1_button = tk.Button(self.window,
          text='RTL', 
          command=self.rtl1_clicked,
          padx=10,
          pady=5,
          width=15
        )
        rtl1_button.grid(column=0, row=8, padx=10, pady=0)

        # land button
        land1_button = tk.Button(self.window,
          text='Land', 
          command=self.land1_clicked,
          padx=10,
          pady=5,
          width=15
        )
        land1_button.grid(column=0, row=9, padx=10, pady=0)
        
        ## second drone
        # guided button
        guided2_button = tk.Button(self.window,
          text='Guided-mode', 
          command=self.guided2_clicked,
          padx=10,
          pady=5,
          width=15
        )
        guided2_button.grid(column=1, row=1, padx=10, pady=0)

        # arm button
        arm2_button = tk.Button(self.window,
          text='Arm', 
          command=self.arm2_clicked,
          padx=10,
          pady=5,
          width=15
        )
        arm2_button.grid(column=1, row=2, padx=10, pady=0)

        # create takeoff button
        takeoff2_button = tk.Button(self.window, 
          text='Takeoff', 
          command=self.takeoff2_clicked,
          padx=10,
          pady=5,
          width=15
        )
        takeoff2_button.grid(column=1, row=3, padx=10, pady=0)

        # start moving button
        start2_button = tk.Button(self.window, 
          text='Start', 
          command=self.start2_clicked,
          padx=10,
          pady=5,
          width=15
        )
        start2_button.grid(column=1, row=4, padx=10, pady=0)

        # left turning button
        left2_button = tk.Button(self.window, 
          text='Left-turning', 
          command=self.left2_clicked,
          padx=10,
          pady=5,
          width=15
        )
        left2_button.grid(column=1, row=5, padx=10, pady=0)

        # right turning button
        right2_button = tk.Button(self.window, 
          text='Right-turning', 
          command=self.right2_clicked,
          padx=10,
          pady=5,
          width=15
        )
        right2_button.grid(column=1, row=6, padx=10, pady=0)

        # stop moving button
        stop2_button = tk.Button(self.window, 
          text='Stop', 
          command=self.stop2_clicked,
          padx=10,
          pady=5,
          width=15
        )
        stop2_button.grid(column=1, row=7, padx=10, pady=0)

        # return-to-launch button
        rtl2_button = tk.Button(self.window,
          text='RTL', 
          command=self.rtl2_clicked,
          padx=10,
          pady=5,
          width=15
        )
        rtl2_button.grid(column=1, row=8, padx=10, pady=0)

        # land button
        land2_button = tk.Button(self.window,
          text='Land', 
          command=self.land2_clicked,
          padx=10,
          pady=5,
          width=15
        )
        land2_button.grid(column=1, row=9, padx=10, pady=0)

        ## third drone
        # guided button
        guided3_button = tk.Button(self.window,
          text='Guided-mode', 
          command=self.guided3_clicked,
          padx=10,
          pady=5,
          width=15
        )
        guided3_button.grid(column=2, row=1, padx=10, pady=0)

        # arm button
        arm3_button = tk.Button(self.window,
          text='Arm', 
          command=self.arm3_clicked,
          padx=10,
          pady=5,
          width=15
        )
        arm3_button.grid(column=2, row=2, padx=10, pady=0)

        # create takeoff button
        takeoff3_button = tk.Button(self.window, 
          text='Takeoff', 
          command=self.takeoff3_clicked,
          padx=10,
          pady=5,
          width=15
        )
        takeoff3_button.grid(column=2, row=3, padx=10, pady=0)

        # start moving button
        start3_button = tk.Button(self.window, 
          text='Start', 
          command=self.start3_clicked,
          padx=10,
          pady=5,
          width=15
        )
        start3_button.grid(column=2, row=4, padx=10, pady=0)

        # left turning button
        left3_button = tk.Button(self.window, 
          text='Left-turning', 
          command=self.left3_clicked,
          padx=10,
          pady=5,
          width=15
        )
        left3_button.grid(column=2, row=5, padx=10, pady=0)

        # right turning button
        right3_button = tk.Button(self.window, 
          text='Right-turning', 
          command=self.right3_clicked,
          padx=10,
          pady=5,
          width=15
        )
        right3_button.grid(column=2, row=6, padx=10, pady=0)

        # stop moving button
        stop3_button = tk.Button(self.window, 
          text='Stop', 
          command=self.stop3_clicked,
          padx=10,
          pady=5,
          width=15
        )
        stop3_button.grid(column=2, row=7, padx=10, pady=0)

        # return-to-launch button
        rtl3_button = tk.Button(self.window,
          text='RTL', 
          command=self.rtl3_clicked,
          padx=10,
          pady=5,
          width=15
        )
        rtl3_button.grid(column=2, row=8, padx=10, pady=0)

        # land button
        land3_button = tk.Button(self.window,
          text='Land', 
          command=self.land3_clicked,
          padx=10,
          pady=5,
          width=15
        )
        land3_button.grid(column=2, row=9, padx=10, pady=0)

        self.window.mainloop()

    def switch_mode(self, mode, drone):
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode
        if drone == 1:
            future = self.copter1_client_mode_switch.call_async(req)
        if drone == 2:
            future = self.copter2_client_mode_switch.call_async(req)
        if drone == 3:
            future = self.copter3_client_mode_switch.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def switch_mode_with_timeout(self, desired_mode: int, timeout: rclpy.duration.Duration, drone):
        is_in_desired_mode = False
        start = self.get_clock().now()
        while not is_in_desired_mode:
            result = self.switch_mode(desired_mode, drone)
            # TODO: Handle successful switch or the case that the vehicle is already in expected mode
            is_in_desired_mode = result.mode_sent
            time.sleep(1)
        return is_in_desired_mode

    def arm(self, drone):
        req = CommandBool.Request()
        req.value = True
        if drone == 1:
            future = self.copter1_client_arm.call_async(req)
        if drone == 2:
            future = self.copter2_client_arm.call_async(req)
        if drone == 3:
            future = self.copter3_client_arm.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def arm_with_timeout(self, timeout: rclpy.duration.Duration, drone):
        armed = False
        start = self.get_clock().now()
        while not armed and self.get_clock().now() - start < timeout:
            armed = self.arm(drone).success
            time.sleep(1)
        return armed

    def takeoff(self, alt, drone):
        req = CommandTOL.Request()
        req.altitude = alt
        req.min_pitch = 0.0
        req.yaw = 0.0
        if drone == 1:
            future = self.copter1_client_takeoff.call_async(req)
        if drone == 2:
            future = self.copter2_client_takeoff.call_async(req)
        if drone == 3:
            future = self.copter3_client_takeoff.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def takeoff_with_timeout(self, alt: int, timeout: rclpy.duration.Duration, drone):
        takeoff_success = False
        start = self.get_clock().now()
        while not takeoff_success and self.get_clock().now() - start < timeout:
            result = self.takeoff(alt, drone)
            takeoff_success = result.success
            time.sleep(1)

        return takeoff_success

    def guided1_clicked(self):
        self.switch_mode_with_timeout("GUIDED",rclpy.duration.Duration(seconds=1), 1)

    def arm1_clicked(self):
        self.arm_with_timeout(rclpy.duration.Duration(seconds=1), 1)

    def takeoff1_clicked(self):
        self.takeoff_with_timeout(30.0, rclpy.duration.Duration(seconds=4), 1)

    def start1_clicked(self):
        msg = String()
        msg.data = 'start'
        self.copter1_cmd_publisher.publish(msg)
        rclpy.spin_once(self, timeout_sec=0)

    def left1_clicked(self):
        msg = String()
        msg.data = 'left'
        self.copter1_cmd_publisher.publish(msg)
        rclpy.spin_once(self, timeout_sec=0)

    def right1_clicked(self):
        msg = String()
        msg.data = 'right'
        self.copter1_cmd_publisher.publish(msg)
        rclpy.spin_once(self, timeout_sec=0)

    def stop1_clicked(self):
        msg = String()
        msg.data = 'stop'
        self.copter1_cmd_publisher.publish(msg)
        rclpy.spin_once(self, timeout_sec=0)

    def rtl1_clicked(self):
        self.switch_mode_with_timeout("RTL", rclpy.duration.Duration(seconds=1), 1)

    def land1_clicked(self):
        self.switch_mode_with_timeout("LAND", rclpy.duration.Duration(seconds=1), 1)

    def guided2_clicked(self):
        self.switch_mode_with_timeout("GUIDED",rclpy.duration.Duration(seconds=2), 2)

    def arm2_clicked(self):
        self.arm_with_timeout(rclpy.duration.Duration(seconds=1), 2)

    def takeoff2_clicked(self):
        self.takeoff_with_timeout(30.0, rclpy.duration.Duration(seconds=4), 2)

    def start2_clicked(self):
        msg = String()
        msg.data = 'start'
        self.copter2_cmd_publisher.publish(msg)
        rclpy.spin_once(self, timeout_sec=0)

    def left2_clicked(self):
        msg = String()
        msg.data = 'left'
        self.copter2_cmd_publisher.publish(msg)
        rclpy.spin_once(self, timeout_sec=0)

    def right2_clicked(self):
        msg = String()
        msg.data = 'right'
        self.copter2_cmd_publisher.publish(msg)
        rclpy.spin_once(self, timeout_sec=0)

    def stop2_clicked(self):
        msg = String()
        msg.data = 'stop'
        self.copter2_cmd_publisher.publish(msg)
        rclpy.spin_once(self, timeout_sec=0)

    def rtl2_clicked(self):
        self.switch_mode_with_timeout("RTL", rclpy.duration.Duration(seconds=1), 2)

    def land2_clicked(self):
        self.switch_mode_with_timeout("LAND", rclpy.duration.Duration(seconds=1), 2)

    def guided3_clicked(self):
        self.switch_mode_with_timeout("GUIDED",rclpy.duration.Duration(seconds=2), 3)

    def arm3_clicked(self):
        self.arm_with_timeout(rclpy.duration.Duration(seconds=1), 3)

    def takeoff3_clicked(self):
        self.takeoff_with_timeout(30.0, rclpy.duration.Duration(seconds=4), 3)

    def start3_clicked(self):
        msg = String()
        msg.data = 'start'
        self.copter3_cmd_publisher.publish(msg)
        rclpy.spin_once(self, timeout_sec=0)

    def left3_clicked(self):
        msg = String()
        msg.data = 'left'
        self.copter3_cmd_publisher.publish(msg)
        rclpy.spin_once(self, timeout_sec=0)

    def right3_clicked(self):
        msg = String()
        msg.data = 'right'
        self.copter3_cmd_publisher.publish(msg)
        rclpy.spin_once(self, timeout_sec=0)

    def stop3_clicked(self):
        msg = String()
        msg.data = 'stop'
        self.copter3_cmd_publisher.publish(msg)
        rclpy.spin_once(self, timeout_sec=0)

    def rtl3_clicked(self):
        self.switch_mode_with_timeout("RTL", rclpy.duration.Duration(seconds=1), 3)

    def land3_clicked(self):
        self.switch_mode_with_timeout("LAND", rclpy.duration.Duration(seconds=1), 3)


def main(args=None):
    rclpy.init(args=args)
    node = P2()
    node.gui()
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
