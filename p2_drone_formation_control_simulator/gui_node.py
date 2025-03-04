import rclpy
import time
import tkinter as tk
from rclpy.node import Node
import numpy as np

from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped, Point
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
from geometry_msgs.msg import PoseStamped

#COPTER_MODE_GUIDED = 4 
#TAKEOFF_ALT = 20.0 

class P2(Node):
    def __init__(self):
        super().__init__('p2')

        self.own_position1 = np.zeros(3)
        self.own_position2 = np.zeros(3)
        self.own_position3 = np.zeros(3)

        # publisher should be used to set the velocities
        self.copter1_cmd_publisher = self.create_publisher(String, 'p2/copter1_cmd', 10)
        self.copter2_cmd_publisher = self.create_publisher(String, 'p2/copter2_cmd', 10)
        self.copter3_cmd_publisher = self.create_publisher(String, 'p2/copter3_cmd', 10)

        self.target_position_publisher = self.create_publisher(Point, 'p2/target_position', 10)

        # Subscriber local position copter 1 
        self.pos_own1 = self.create_subscription(PoseStamped,
                                               'iris1/local_position/pose',
                                               lambda msg: self.update_position(self.own_position1, msg),
                                               rclpy.qos.qos_profile_sensor_data)
        # Subscriber local position copter 2 
        self.pos_own2 = self.create_subscription(PoseStamped,
                                               'iris2/local_position/pose',
                                               lambda msg: self.update_position(self.own_position2, msg),
                                               rclpy.qos.qos_profile_sensor_data)
        
        # Subscriber local position copter 3 
        self.pos_own3 = self.create_subscription(PoseStamped,
                                               'iris3/local_position/pose',
                                               lambda msg: self.update_position(self.own_position3, msg),
                                               rclpy.qos.qos_profile_sensor_data)

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

        self.current_position_entry1 = None
        self.current_position_entry2 = None
        self.current_position_entry3 = None


    def update_position(self, position, msg: PoseStamped):
        position[0] = msg.pose.position.x
        position[1] = msg.pose.position.y
        position[2] = msg.pose.position.z

    def update_gui(self):
        entries = [self.current_position_entry1, self.current_position_entry2, self.current_position_entry3]
        positions = [self.own_position1, self.own_position2, self.own_position3]

        for entry, position in zip(entries, positions):
            if not entry:
                continue
            pos_text = f"X: {position[0]:.2f}, Y: {position[1]:.2f}, Z: {position[2]:.2f}"

            # Enable Entry, update text, then set to readonly again
            entry.configure(state='normal')
            entry.delete(0, tk.END)
            entry.insert(0, pos_text)
            entry.configure(state='readonly')
        
        self.window.after(100, self.update_gui)

    def gui(self):
        self.window = tk.Tk()
        self.window.title('P2 Simulator')

        l1 = tk.Label(self.window, text = "Copter 1")
        l2 = tk.Label(self.window, text = "Copter 2")
        l3 = tk.Label(self.window, text = "Copter 3")
        l4 = tk.Label(self.window, text = "All Copters")
        l1.grid(column=0, row=0, pady=0)
        l2.grid(column=1, row=0, pady=0)
        l3.grid(column=2, row=0, pady=0)
        l4.grid(column=3, row=0, pady=0)

            ## first drone
        # guided button


        #currentpostiond1 = tk.Entry(self.window)
        #currentpostiond1.grid(column=0, row=10, padx=10, pady=0)
        #currentpostiond1.insert(0, self.get_logger().info('Position = ' + str(self.own_position)))  # Set initial text
        #currentpostiond1.configure(state='readonly')  # Make it read-only

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

        self.current_position_entry1 = tk.Entry(self.window, width=30)
        self.current_position_entry1.grid(column=0, row=10, padx=10, pady=0)
        self.current_position_entry1.configure(state='readonly')
        
            ## Second drone
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

        self.current_position_entry2 = tk.Entry(self.window, width=30)
        self.current_position_entry2.grid(column=1, row=10, padx=10, pady=0)
        self.current_position_entry2.configure(state='readonly')


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

        self.current_position_entry3 = tk.Entry(self.window, width=30)
        self.current_position_entry3.grid(column=2, row=10, padx=10, pady=0)
        self.current_position_entry3.configure(state='readonly')

        # guided button
        guided4_button = tk.Button(self.window,
                                   text='Guided-mode',
                                   command=self.guided4_clicked,
                                   padx=10,
                                   pady=5,
                                   width=15
                                   )
        guided4_button.grid(column=3, row=1, padx=10, pady=0)

        # arm button
        arm4_button = tk.Button(self.window,
                                text='Arm',
                                command=self.arm4_clicked,
                                padx=10,
                                pady=5,
                                width=15
                                )
        arm4_button.grid(column=3, row=2, padx=10, pady=0)

        # create takeoff button
        takeoff4_button = tk.Button(self.window,
                                    text='Takeoff',
                                    command=self.takeoff4_clicked,
                                    padx=10,
                                    pady=5,
                                    width=15
                                    )
        takeoff4_button.grid(column=3, row=3, padx=10, pady=0)

        # start moving button
        start4_button = tk.Button(self.window,
                                  text='Start',
                                  command=self.start4_clicked,
                                  padx=10,
                                  pady=5,
                                  width=15
                                  )
        start4_button.grid(column=3, row=4, padx=10, pady=0)

        # left turning button
        left4_button = tk.Button(self.window,
                                 text='Left-turning',
                                 command=self.left4_clicked,
                                 padx=10,
                                 pady=5,
                                 width=15
                                 )
        left4_button.grid(column=3, row=5, padx=10, pady=0)

        # right turning button
        right4_button = tk.Button(self.window,
                                  text='Right-turning',
                                  command=self.right4_clicked,
                                  padx=10,
                                  pady=5,
                                  width=15
                                  )
        right4_button.grid(column=3, row=6, padx=10, pady=0)

        # stop moving button
        stop4_button = tk.Button(self.window,
                                 text='Stop',
                                 command=self.stop4_clicked,
                                 padx=10,
                                 pady=5,
                                 width=15
                                 )
        stop4_button.grid(column=3, row=7, padx=10, pady=0)

        # return-to-launch button
        rtl4_button = tk.Button(self.window,
                                text='RTL',
                                command=self.rtl4_clicked,
                                padx=10,
                                pady=5,
                                width=15
                                )
        rtl4_button.grid(column=3, row=8, padx=10, pady=0)

        # land button
        land4_button = tk.Button(self.window,
                                 text='Land',
                                 command=self.land4_clicked,
                                 padx=10,
                                 pady=5,
                                 width=15
                                 )
        land4_button.grid(column=3, row=9, padx=10, pady=0)

        formation_button = tk.Button(self.window,
                                 text='Formation',
                                 command=self.formation_clicked,
                                 padx=10,
                                 pady=5,
                                 width=15
                                 )
        formation_button.grid(column=3, row=10, padx=10, pady=0)

        ## Enter Target Position coordinates in toprightmostcorner

        tk.Label(text="Enter Target Position Copter 1").grid(column=4, row=0, padx=5, pady=5)


        tk.Label(self.window, text="Target X:").grid(column=4, row=1, padx=5, pady=5)
        tk.Label(self.window, text="Target Y:").grid(column=4, row=2, padx=5, pady=5)
        tk.Label(self.window, text="Target Z:").grid(column=4, row=3, padx=5, pady=5)


        self.target_x_entry = tk.Entry(self.window, width=10)
        self.target_y_entry = tk.Entry(self.window, width=10)
        self.target_z_entry = tk.Entry(self.window, width=10)

        self.target_x_entry.grid(column=5, row=1, padx=5, pady=5)
        self.target_y_entry.grid(column=5, row=2, padx=5, pady=5)
        self.target_z_entry.grid(column=5, row=3, padx=5, pady=5)

        movetotarget_button = tk.Button(self.window,
          text='Move-to-Target', 
          command=self.movetotarget_clicked,
          padx=10,
          pady=5,
          width=15
        )
        movetotarget_button.grid(column=4, row=4, padx=10, pady=0)

        self.update_gui()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            self.window.update_idletasks()
            self.window.update()

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
     

    def movetotarget_clicked(self):
        msg = String()
        msg.data = 'movetotarget'
        self.copter1_cmd_publisher.publish(msg)

        msg = Point()
        msg.x = float(self.target_x_entry.get())
        msg.y = float(self.target_y_entry.get())
        msg.z = float(self.target_z_entry.get())
        self.target_position_publisher.publish(msg)

        rclpy.spin_once(self, timeout_sec=0)

    def guided1_clicked(self):
        self.switch_mode_with_timeout("GUIDED",rclpy.duration.Duration(seconds=1), 1)

    def arm1_clicked(self):
        self.arm_with_timeout(rclpy.duration.Duration(seconds=1), 1)

    def takeoff1_clicked(self):
        self.takeoff_with_timeout(30.0, rclpy.duration.Duration(seconds=2), 1)

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
        self.takeoff_with_timeout(30.0, rclpy.duration.Duration(seconds=2), 2)

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
        self.takeoff_with_timeout(30.0, rclpy.duration.Duration(seconds=2), 3)

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

    def guided4_clicked(self):
        self.switch_mode_with_timeout("GUIDED", rclpy.duration.Duration(seconds=1), 1)
        self.switch_mode_with_timeout("GUIDED", rclpy.duration.Duration(seconds=1), 2)
        self.switch_mode_with_timeout("GUIDED", rclpy.duration.Duration(seconds=1), 3)

    def arm4_clicked(self):
        self.arm(1)
        self.arm(2)
        self.arm(3)

    def takeoff4_clicked(self):
        self.takeoff_with_timeout(30.0, rclpy.duration.Duration(seconds=2), 1)
        self.takeoff_with_timeout(30.0, rclpy.duration.Duration(seconds=2), 2)
        self.takeoff_with_timeout(30.0, rclpy.duration.Duration(seconds=2), 3)

    def start4_clicked(self):
        msg = String()
        msg.data = 'start'
        self.copter1_cmd_publisher.publish(msg)
        self.copter2_cmd_publisher.publish(msg)
        self.copter3_cmd_publisher.publish(msg)
        rclpy.spin_once(self, timeout_sec=0)

    def left4_clicked(self):
        msg = String()
        msg.data = 'left'
        self.copter1_cmd_publisher.publish(msg)
        self.copter2_cmd_publisher.publish(msg)
        self.copter3_cmd_publisher.publish(msg)
        rclpy.spin_once(self, timeout_sec=0)

    def right4_clicked(self):
        msg = String()
        msg.data = 'right'
        self.copter1_cmd_publisher.publish(msg)
        self.copter2_cmd_publisher.publish(msg)
        self.copter3_cmd_publisher.publish(msg)
        rclpy.spin_once(self, timeout_sec=0)

    def stop4_clicked(self):
        msg = String()
        msg.data = 'stop'
        self.copter1_cmd_publisher.publish(msg)
        self.copter2_cmd_publisher.publish(msg)
        self.copter3_cmd_publisher.publish(msg)
        rclpy.spin_once(self, timeout_sec=0)

    def rtl4_clicked(self):
        self.switch_mode_with_timeout("RTL", rclpy.duration.Duration(seconds=1), 1)
        self.switch_mode_with_timeout("RTL", rclpy.duration.Duration(seconds=1), 2)
        self.switch_mode_with_timeout("RTL", rclpy.duration.Duration(seconds=1), 3)

    def land4_clicked(self):
        self.switch_mode_with_timeout("LAND", rclpy.duration.Duration(seconds=1), 1)
        self.switch_mode_with_timeout("LAND", rclpy.duration.Duration(seconds=1), 2)
        self.switch_mode_with_timeout("LAND", rclpy.duration.Duration(seconds=1), 3)

    def formation_clicked(self):
        msg = String()
        msg.data = 'formation'
        self.copter2_cmd_publisher.publish(msg)
        self.copter3_cmd_publisher.publish(msg)
        rclpy.spin_once(self, timeout_sec=0)


def main(args=None):
    rclpy.init(args=args)
    node = P2()
    node.gui()
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
