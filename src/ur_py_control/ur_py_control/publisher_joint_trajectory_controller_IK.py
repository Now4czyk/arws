# Copyright 2023 Poznan University of Technology
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Dominik Belter, Rafal Staszak
#

import rclpy
import time

from rclpy.node import Node
from builtin_interfaces.msg import Duration
from rcl_interfaces.msg import ParameterDescriptor

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

  
class ComputeIKAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')

        #initialize inverse kinematic client
        self.cli = self.create_client(GetPositionIK, 'compute_ik')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetPositionIK.Request()

        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
          'base_link', 'object').get_parameter_value().string_value

        # initialize transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.on_timer_tf)
        self.timer2 = self.create_timer(1.0, self.on_timer_control)
        self.get_logger().info(f'Timer created')

        #initialize ik
        self.is_pose = False
        self.goal_pose = TransformStamped()
        self.client_futures = []

        #initialize publisher (control joints of the robot)
        controller_name = "joint_trajectory_controller"
        wait_sec_between_publish = 6.0
        self.joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                       'wrist_3_joint']

        publish_topic = "/" + controller_name + "/" + "joint_trajectory"

        self.publisher_ = self.create_publisher(JointTrajectory, publish_topic, 1)
        self.ref_positions = []

    def on_timer_tf(self):
        # Store frame names in variables that will be used to
        # compute transformations
        to_frame_rel = 'base_link'
        from_frame_rel = 'object'

        # Look up for the transformation between target_frame and turtle2 frames
        # and send velocity commands for turtle2 to reach target_frame
        self.get_logger().info(f'Read transform')
        try:
            t = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, rclpy.time.Time())
            self.get_logger().info(f'Transform\n {t}')
            self.goal_pose = t
            self.goal_pose.transform._translation._z += 0.1
            self.is_pose = True
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
        return
    
    def on_timer_control(self):
        self.send_request()
        return
    
    def send_request(self):
        if not self.is_pose:
            self.get_logger().info('waiting for goal pose...')
        else :
            self.get_logger().info(f'Send request')
            self.req.ik_request.pose_stamped.pose.position.x = self.goal_pose.transform.translation.x
            self.req.ik_request.pose_stamped.pose.position.y = self.goal_pose.transform.translation.y
            self.req.ik_request.pose_stamped.pose.position.z = self.goal_pose.transform.translation.z
            self.req.ik_request.pose_stamped.pose.orientation = self.goal_pose.transform.rotation
            self.req.ik_request.group_name = "ur_manipulator"
            self.req.ik_request.ik_link_name = "tool0"
            self.get_logger().info(f'Request:\n {self.req}')
          
            self.client_futures.append(self.cli.call_async(self.req))
        return
    
    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            incomplete_futures = []
            for f in self.client_futures:
                if f.done():
                    res = f.result()
                    # rclpy.spin_until_future_complete(self, self.future)
                    self.get_logger().info(f'Response:\n {res}')
                    self.get_logger().info(f'Moveit Error code:\n {res.error_code}')
                    if (res.error_code.val == -31):
                        self.get_logger().info(f'No IK solution')
                    else :
                        self.ref_positions.append(res.solution.joint_state.position)
                        self.positions = self.ref_positions
                        # Read all positions from parameters
                        self.goals = []  # List of JointTrajectoryPoint
                        for pos in self.positions:
                            point = JointTrajectoryPoint()
                            point.positions = pos

                            point.time_from_start = Duration(sec=4)
                            self.goals.append(point)
                            self.get_logger().info(f'Goal  has definition {point}')
                            
                        traj = JointTrajectory()
                        traj.joint_names = self.joints
                        traj.points.append(self.goals[0])
                        self.publisher_.publish(traj)
                    self.get_logger().info(f'IK result:\n {res}')
                    return res
                else:
                    incomplete_futures.append(f)
            self.client_futures = incomplete_futures
    
def main(args=None):
    rclpy.init(args=args)

    client_tf = ComputeIKAsync()
    client_tf.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
