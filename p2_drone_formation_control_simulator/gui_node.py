import rclpy
import time
import tkinter as tk
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from ardupilot_msgs.srv import ArmMotors
from ardupilot_msgs.srv import ModeSwitch
from ardupilot_msgs.srv import Takeoff

#COPTER_MODE_GUIDED = 4 
#TAKEOFF_ALT = 20.0 

class P2(Node):
    def __init__(self):
        super().__init__('p2')

        # publisher should be used to set the velocities
        self.cmd_publisher = self.create_publisher(String, 'p2/cmd', 10)

        # clients
        self.declare_parameter("mode_topic", "/ap/mode_switch")
        self._mode_topic = self.get_parameter("mode_topic").get_parameter_value().string_value
        self._client_mode_switch = self.create_client(ModeSwitch, self._mode_topic)
        while not self._client_mode_switch.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('mode switch service not available, waiting again...')

        self.declare_parameter("arm_topic", "/ap/arm_motors")
        self._arm_topic = self.get_parameter("arm_topic").get_parameter_value().string_value
        self._client_arm = self.create_client(ArmMotors, self._arm_topic)
        while not self._client_arm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arm service not available, waiting again...')

        self.declare_parameter("takeoff_service", "/ap/experimental/takeoff")
        self._takeoff_topic = self.get_parameter("takeoff_service").get_parameter_value().string_value
        self._client_takeoff = self.create_client(Takeoff, self._takeoff_topic)
        while not self._client_takeoff.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('takeoff service not available, waiting again...')

    def gui(self):
        self.window = tk.Tk()
        self.window.title('P2 Simulator')


        # guided button
        guided_button = tk.Button(self.window,
          text='Guided-mode', 
          command=self.guided_clicked,
          padx=10,
          pady=5,
          width=15
        )
        guided_button.pack(padx=10, pady=0)

        # arm button
        arm_button = tk.Button(self.window,
          text='Arm', 
          command=self.arm_clicked,
          padx=10,
          pady=5,
          width=15
        )
        arm_button.pack(padx=10, pady=0)

        # create takeoff button
        takeoff_button = tk.Button(self.window, 
          text='Takeoff', 
          command=self.takeoff_clicked,
          padx=10,
          pady=5,
          width=15
        )
        takeoff_button.pack(padx=10, pady=0)

        # start moving button
        start_button = tk.Button(self.window, 
          text='Start', 
          command=self.start_clicked,
          padx=10,
          pady=5,
          width=15
        )
        start_button.pack(padx=10, pady=0)

        # left turning button
        left_button = tk.Button(self.window, 
          text='Left-turning', 
          command=self.left_clicked,
          padx=10,
          pady=5,
          width=15
        )
        left_button.pack(padx=10, pady=0)

        # right turning button
        right_button = tk.Button(self.window, 
          text='Right-turning', 
          command=self.right_clicked,
          padx=10,
          pady=5,
          width=15
        )
        right_button.pack(padx=10, pady=0)

        # stop moving button
        stop_button = tk.Button(self.window, 
          text='Stop', 
          command=self.stop_clicked,
          padx=10,
          pady=5,
          width=15
        )
        stop_button.pack(padx=10, pady=0)

        # return-to-launch button
        rtl_button = tk.Button(self.window,
          text='RTL', 
          command=self.rtl_clicked,
          padx=10,
          pady=5,
          width=15
        )
        rtl_button.pack(padx=10, pady=0)

        # land button
        land_button = tk.Button(self.window,
          text='Land', 
          command=self.land_clicked,
          padx=10,
          pady=5,
          width=15
        )
        land_button.pack(padx=10, pady=0)

        self.window.mainloop()

    def switch_mode(self, mode):
        req = ModeSwitch.Request()
        assert mode in [4, 6, 9]
        req.mode = mode
        future = self._client_mode_switch.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def switch_mode_with_timeout(self, desired_mode: int, timeout: rclpy.duration.Duration):
        is_in_desired_mode = False
        start = self.get_clock().now()
        while not is_in_desired_mode:
            result = self.switch_mode(desired_mode)
            # Handle successful switch or the case that the vehicle is already in expected mode
            is_in_desired_mode = result.status or result.curr_mode == desired_mode
            time.sleep(1)

        return is_in_desired_mode

    def arm(self):
        req = ArmMotors.Request()
        req.arm = True
        future = self._client_arm.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def arm_with_timeout(self, timeout: rclpy.duration.Duration):
        armed = False
        start = self.get_clock().now()
        while not armed and self.get_clock().now() - start < timeout:
            armed = self.arm().result
            time.sleep(1)
        return armed

    def takeoff(self, alt):
        req = Takeoff.Request()
        req.alt = alt
        future = self._client_takeoff.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def takeoff_with_timeout(self, alt: int, timeout: rclpy.duration.Duration):
        takeoff_success = False
        start = self.get_clock().now()
        while not takeoff_success:
            result = self.takeoff(alt)
            takeoff_success = result.status
            time.sleep(1)

        return takeoff_success

    def guided_clicked(self):
        self.switch_mode_with_timeout(4, rclpy.duration.Duration(seconds=1))

    def arm_clicked(self):
        self.arm_with_timeout(rclpy.duration.Duration(seconds=1))

    def takeoff_clicked(self):
        self.takeoff_with_timeout(20.0, rclpy.duration.Duration(seconds=20))

    def start_clicked(self):
        msg = String()
        msg.data = 'start'
        self.cmd_publisher.publish(msg)
        rclpy.spin_once(self, timeout_sec=0)

    def left_clicked(self):
        msg = String()
        msg.data = 'left'
        self.cmd_publisher.publish(msg)
        rclpy.spin_once(self, timeout_sec=0)

    def right_clicked(self):
        msg = String()
        msg.data = 'right'
        self.cmd_publisher.publish(msg)
        rclpy.spin_once(self, timeout_sec=0)

    def stop_clicked(self):
        msg = String()
        msg.data = 'stop'
        self.cmd_publisher.publish(msg)
        rclpy.spin_once(self, timeout_sec=0)

    def rtl_clicked(self):
        self.switch_mode_with_timeout(6, rclpy.duration.Duration(seconds=1))

    def land_clicked(self):
        self.switch_mode_with_timeout(9, rclpy.duration.Duration(seconds=1))

def main(args=None):
    rclpy.init(args=args)
    node = P2()
    node.gui()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
