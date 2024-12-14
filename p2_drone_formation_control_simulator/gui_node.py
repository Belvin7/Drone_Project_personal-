import rclpy
import tkinter as tk
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped


class P2(Node):

    def __init__(self):
        super().__init__('p2')
        self.cmd_publisher = self.create_publisher(String, 'p2/cmd', 10)

    def gui(self):
        self.window = tk.Tk()
        self.window.title('P2 Simulator')

        # create cmd button
        launch_button = tk.Button(self.window, 
                  text='launch', 
                  command=self.launch_clicked,
                  padx=10,
                  pady=5,
                  width=15
        )
        launch_button.pack(padx=10, pady=0)

        # return-to-home button
        rtl_button = tk.Button(self.window,
                  text='RTL', 
                  command=self.rtl_clicked,
                  padx=10,
                  pady=5,
                  width=15
        )
        rtl_button.pack(padx=10, pady=0)

        self.window.mainloop()
        
    def launch_clicked(self):
       msg = String()
       msg.data = 'launch'
       self.cmd_publisher.publish(msg)
       rclpy.spin_once(self, timeout_sec=0)

    def rtl_clicked(self):
       msg = String()
       msg.data = 'rtl'
       self.cmd_publisher.publish(msg)
       rclpy.spin_once(self, timeout_sec=0)


def main(args=None):
    rclpy.init(args=args)
    node = P2()
    node.gui()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
