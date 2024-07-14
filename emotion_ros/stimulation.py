#!/usr/bin/env python3
# -*- coding: utf-8 -*- 

import rclpy
from rclpy.node import Node
import sys
import time
import os
import psutil
import filecmp
import cv2
from std_msgs.msg import Int32

class Stimulation(Node):
  def __init__(self):
    super().__init__('stimulation')

    self.create_subscription(Int32, 'time_count', self.stimu_callback, 10)
    
  def stimu_callback(self, msg):
    self.get_logger().info(f'RECV_TIME_COUNT {msg.data}')
    
    if msg.data >= 30: # 31?
      # 刺激提示
      return
    if msg.data == 60: # 61?
      # 終わらせる
      return
    # 刺激提示

def main(args=None):
  try:
    rclpy.init(args=args)
    talker = Stimulation()
    rclpy.spin(talker)
  except KeyboardInterrupt:
    pass
  finally:
    talker.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()

