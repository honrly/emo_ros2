#!/usr/bin/env python3
# -*- coding: utf-8 -*- 

import rclpy
from rclpy.node import Node
import sys
import csv
from datetime import datetime
import os
import psutil
import filecmp
import cv2
from std_msgs.msg import Int32
import keyboard

class UserEnter(Node):
  def __init__(self):
    super().__init__('user_enter')

    # self.pub_user_enter = self.create_publisher(Int32, 'user_enter', 10)
    
    # Record csv
    # directory_path = '/home/user/ros2_ws/src/emotion_ros'
    directory_path = '/home/user/turtlebot3_ws/src/emotion_ros'
      
    enter_data_path = os.path.join(directory_path, 'bio_record/enter_data')

    os.makedirs(enter_data_path, exist_ok=True)

    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

    self.csv_filename = os.path.join(enter_data_path, f'{timestamp}_enter.csv')
    self.csv_file = open(self.csv_filename, mode='w', newline='')
    self.csv_writer = csv.writer(self.csv_file)
    self.csv_writer.writerow(['timestamp'])

    
  def check_for_enter_key(self):
    if keyboard.is_pressed('enter'):        
      current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
      self.get_logger().info(f'Enter key pressed at {current_time}')
      
      self.csv_writer.writerow([current_time])
      self.csv_file.flush()
  
  def run(self):
    while rclpy.ok():
      self.check_for_enter_key()

def main(args=None):
  rclpy.init(args=args)
  talker = UserEnter()
  try:
      talker.run()
  except KeyboardInterrupt:
    pass
  finally:
    talker.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()

