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
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
import sys

import numpy as np
import cv2
from cv_bridge import CvBridge
angle = 0.0

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_video_subscriber')

        self.subscription = self.create_subscription(Point,'topic',self.listener_callback,10)
        self.subscription  # prevent unused variable warning

        # Set up QoS Profiles for passing LIDAR info over WiFi
        lidar_qos_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history = QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            durability = QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            depth = 1
        )

        #Declare that the minimal_subscriber node is subscribing to the /scan topic.
        self._minimal_subscriber = self.create_subscription(LaserScan,'/scan',self.listener_lidar,lidar_qos_profile)
        self._minimal_subscriber  # prevent unused variable warning

        #Declare publisher for angle and depth
        self.publisher1_ = self.create_publisher(Float32MultiArray,'angle_depth',10)


    def listener_callback(self, msg):
        # Convert horizontal coordinates to horizontal angle
        coordinates_x = msg.x
        global angle
        angle = ((coordinates_x/ 320) * 62.2)
   
    def listener_lidar(self,msg):
        depth1_index = (round((len(msg.ranges)/360) * angle))
        depth1 = (msg.ranges[depth1_index])
        new_angle = angle - 31.1
        new_message = Float32MultiArray(data = [new_angle, depth1])
        self.publisher1_.publish(new_message)
        #self.get_logger().info('Publishing: "%s"' %str(new_message))
    

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