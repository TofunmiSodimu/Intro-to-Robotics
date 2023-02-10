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
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
import sys

import numpy as np
import cv2
from cv_bridge import CvBridge

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_video_subscriber')
        self.subscription = self.create_subscription(
            Point,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        #Declare publisher
        self._vel_publish = self.create_publisher(Twist,'/cmd_vel', 10)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % str(msg))
        coordinates_x = msg.x
        # Move to left
        if (coordinates_x > 180) and (coordinates_x <= 230):
            move = Twist()
            move.angular.z = -0.1
            self._vel_publish.publish(move)
            #self.get_logger().info('Publishing: "%s"' %str(message))

        # Stay at center
        if (coordinates_x < 180) and (coordinates_x >= 140):
            move = Twist()
            move.angular.z = 0.0
            self._vel_publish.publish(move)
            #self.get_logger().info('Publishing: "%s"' %str(message))

        # Move to right
        elif(coordinates_x < 140) and (coordinates_x >= 80):
            move = Twist()
            move.angular.z = 0.1
            self._vel_publish.publish(move)
            #self.get_logger().info('Publishing: "%s"' %str(message))


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
