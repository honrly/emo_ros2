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

class Stimu(Node):
  def __init__(self):
    super().__init__('stimu')

    self.create_subscription(Int32, 'time_count', self.stimu_callback, 10)
    
def stimu_callback(self, msg):
    if msg.data == 30:
        return

def main(args=None):
  try:
    rclpy.init(args=args)
    talker = Stimu()
    rclpy.spin(talker)
  except KeyboardInterrupt:
    pass
  finally:
    talker.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()

