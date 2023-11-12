#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
import cv2
from cv_bridge import CvBridge, CvBridgeError
import threading
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
import matplotlib.pyplot as plt


class ShowJointStates(Node):
    def __init__(self):
        super().__init__('show_joint_states')
        self.subscriber_ = self.create_subscription(JointState, '/joint_states', self.get_state,
                                                    qos_profile=qos_profile_sensor_data)

        self.how_long = 15.0  # seconds
        self.state = {}

    def get_state(self, msg: JointState):
        for position, name in zip(msg.position, msg.name):
            if name not in self.state.keys():
                self.state[name] = [position]
            else:
                self.state[name].append(position)

    def run(self):
        first_tick = self.get_clock().now()
        while self.get_clock().now() - first_tick < Duration(seconds=self.how_long):
            pass

        if self.state:  # if data was collected
            fig, axs = plt.subplots(len(self.state))

            fig.suptitle('Joint states')
            for i, name in enumerate(self.state.keys()):
                axs[i].plot(self.state[name])
                axs[i].set_title(name)
            fig.tight_layout()
            plt.show()


def main(args=None):
    rclpy.init(args=args)

    node = ShowJointStates()

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    node.run()

    node.destroy_node()
    rclpy.shutdown()
    thread.join()


if __name__ == '__main__':
    main()
