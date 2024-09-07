#!/usr/bin/env python3
# -*- coding: utf-8 -*- 

import rclpy
from rclpy.node import Node
import sys
import time
import os
import psutil
import filecmp
import threading
import cv2
from std_msgs.msg import String
import csv
from datetime import datetime
from zoneinfo import ZoneInfo

class FaceCtrl(Node):
  def __init__(self):
    super().__init__('face_ctrl')
    
    self.target_image = ""
    self.stop_flg = 0
    self.home_dir = os.environ["HOME"]

    #self.disp("initial")
    os.system("setterm -cursor off>/dev/tty0")
    self.disp("BLACK")
    
    t = threading.Thread(args=(), target=self.disp_thread)
    t.start()
    self.create_subscription(String, 'face', self.callback, 10)
    
    # Record csv
    # directory_path = '/home/user/ros2_ws/src/emotion_ros'
    directory_path = '/home/user/turtlebot3_ws/src/emotion_ros'
      
    face_data_path = os.path.join(directory_path, 'data_record/face_data')
    os.makedirs(face_data_path, exist_ok=True)

    timestamp = datetime.now(ZoneInfo("Asia/Tokyo")).strftime('%Y%m%d_%H%M%S')

    self.csv_filename = os.path.join(face_data_path, f'{timestamp}_fix_num.csv')
    self.csv_file = open(self.csv_filename, mode='w', newline='')
    self.csv_writer = csv.writer(self.csv_file)
    self.csv_writer.writerow(['timestamp', 'face'])

  def write_face_data(self, face):
    timestamp = datetime.now(ZoneInfo("Asia/Tokyo")).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
    self.csv_writer.writerow([timestamp, face])
    self.csv_file.flush()
    

  def cmp_file(self, fn1, fn2):
    f1 = open(fn1, 'rb')
    f2 = open(fn2, 'rb')

    string1 = f1.read()
    string2 = f2.read()

    if string1 == string2: return True
    return False

  ### 採用版 ###
  def disp(self, img_file):
    command1 = "cat " + self.home_dir + "/turtlebot3_ws/src/emotion_ros/face_DB/" + img_file + ".raw > /dev/fb0"
    command2 = "cat /dev/fb0 > /home/ubuntu/tmp.raw"
    os.system(command1)
   
  def disp_cv(self, img_file):
    fff = self.home_dir + "/turtlebot3_ws/src/emotion_ros/face_DB/" + img_file + ".JPG"
    img = cv2.imread(fff)
    img_resize = cv2.resize(img, (320,180))
    cv2.imshow("Face", img_resize)
    cv2.waitKey(1)
    #self.get_logger().info("\n###\n### Disp " + str(fff) + "\n###")

  def disp_thread(self):
    self.get_logger().info("######### disp_thread #########")
    while 1:
        if self.stop_flg:
            break
        if self.target_image == "":
            continue
        self.disp(self.target_image)
        #self.disp_cv(self.target_image)
        time.sleep(0.1)      

  def callback(self, msg):
    self.write_face_data(msg.data)
    self.get_logger().info("Message " + str(msg.data) + " recieved")
    self.target_image = msg.data

def main(args=None):
  try:
    rclpy.init(args=args)
    talker = FaceCtrl()
    rclpy.spin(talker)
    talker.stop_flg = 1
  except KeyboardInterrupt:
    pass
  finally:
    # os.system("setterm -cursor on>/dev/tty0")
    talker.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()
