#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# this is how to import custom messages
from ur_custom_interfaces.msg import URCommand

class CameraNode (Node):
    def __init__ (self):
        super().__init__("py_test")
        self.counter_ = 0
        self.get_logger ().info("Hello ROS2")
        self.create_timer (0.5, self.timer_callback)
        self.publisher_ = self.create_publisher(URCommand, "custom_camera", 1)
    
    def timer_callback(self):
        self.counter_ += 1
        msg = URCommand()
        msg.command = "command" + str(self.counter_)
        msg.x = "x:" + str(self.counter_)
        msg.y = "y:" + str(self.counter_)
        self.publisher_.publish(msg)
        self.get_logger().info("Hello " + str(self.counter_))

def main (args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()