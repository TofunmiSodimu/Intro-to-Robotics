# Copyright 2016 Open Source Robotics Foundation, Inc.
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

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import sys

import numpy as np
import cv2
from cv_bridge import CvBridge

class MinimalChaseObject(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        # Declare subscriber
        self.subscription1 = self.create_subscription(Float32MultiArray,'angle_depth',self.listener_callback,10)
        self.subscription1  # prevent unused variable warning

        #Declare publisher
        self._vel_publish = self.create_publisher(Twist,'/cmd_vel', 10)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % str(msg))
        angle = msg.data[0]
        depth1 = msg.data[1]
        previous_angle = 0.0
        previous_depth = 0.0

        # Move to left and forward
        if ((angle-previous_angle) > 2.0):
            move = Twist()
            move.angular.z = -0.1
            if depth1 > 0.5:
                move.linear.x = 0.05
                move.linear.y = 0.05
            self._vel_publish.publish(move)

        # Stay at center
        elif (angle > -2) and (angle < 2):
            move = Twist()
            move.angular.z = 0.0
            if depth1 > 0.5:
                move.linear.x = 0.05
                move.linear.y = 0.05
            self._vel_publish.publish(move)
            #self.get_logger().info('Publishing: "%s"' %str(message))

        # Move to right and forward
        elif ((angle-previous_angle) < 2.0):
            move = Twist()
            move.angular.z = 0.1
            if depth1 > 0.5:
                move.linear.x = 0.05
                move.linear.y = 0.05
            self._vel_publish.publish(move)

        else:
            move.angular.z = 0.0
            move.linear.x = 0.0
            move.linear.y = 0.0

def main(args=None):
    rclpy.init(args=args)

    minimal_chase = MinimalChaseObject()

    rclpy.spin(minimal_chase)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    move = Twist()
    move.angular.z = 0.0
    move.linear.x = 0.1
    move.linear.y = 0.1
    minimal_chase._vel_publish.publish(move)
    minimal_chase.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()