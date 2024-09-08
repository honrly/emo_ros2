#!/usr/bin/env python3
# -*- coding: utf-8 -*- 

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import time
import threading
import math
import numpy as np
from nav_msgs.msg import Odometry
import tf_transformations
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan   #message from laser range finder
from std_msgs.msg import String

class MotionCtrl(Node):
  def __init__(self):
    super().__init__('motion_ctrlb_rest')
    
    self.mode = 1 # motion switch
    self.old_face = '' # previous emotion
    
    ### scoring ###
    self.recv_max = 5
    self.recv_cnt = 0
    self.em_score = [0, 0, 0, 0]
    self.rep_cnt = 0
    self.rep_max = 2

    ### speech ###
    self.snd_happy   = ["Happy.wav"]
    self.snd_relax   = ["Relax.wav"]
    self.snd_anger   = ["Nerves.wav", "Deep_B.wav"]
    self.snd_sadness = ["Deep_B.wav", "Ok.wav", "Cheer_up.wav", "Keep_up.wav"]
    self.cnt_happy = 0
    self.cnt_relax = 0
    self.cnt_anger = 0
    self.cnt_sadness = 0

    ### subscriver ###
    self.create_subscription(String, 'emo_status', self.callback_emotion, 10)

    ### publisher ###
    self.pub_emotion2 = self.create_publisher(String, 'emotion2', 10)
    self.pub_face = self.create_publisher(String, 'face', 10)
    self.pub_sound = self.create_publisher(String, 'speech', 10)
    self.pub_motion = self.create_publisher(String, 'motion', 10)

  def get_next(self, cnt, maxcnt):
    n = cnt + 1
    if n >= maxcnt:
        n = 0
    return n

  def callback_emotion(self, msg):
    face = msg.data
    self.get_logger().info("Message " + str(msg.data) + " recieved")
    tmp = String()

    # 実際の応答を実行
    if face == 'Happy':
        self.get_logger().info("\n#############\n### Happy ###\n#############")
        tmp.data = "happy5"
        self.pub_face.publish(tmp)
        tmp.data = self.snd_happy[self.cnt_happy]
        self.pub_sound.publish(tmp)
        self.cnt_happy = self.get_next(self.cnt_happy, len(self.snd_happy))
        if self.mode == 1:
            #tmp.data = "FB2,0.05,1.0,0.1,0"
            tmp.data = "LR,0.2,2.0,0.6,0"
            self.pub_motion.publish(tmp)
    if face == 'Relax':
        self.get_logger().info("\n#############\n### Relax ###\n#############")
        tmp.data = "relax5"
        self.pub_face.publish(tmp)
        tmp.data = self.snd_relax[self.cnt_relax]
        self.pub_sound.publish(tmp)
        self.cnt_relax = self.get_next(self.cnt_relax, len(self.snd_relax))
        if self.mode == 1:
            tmp.data = "LR,0.0,2.0,0.3,0"
            self.pub_motion.publish(tmp)
    if face == 'Anger':
        self.get_logger().info("\n#############\n### Anger ###\n#############")
        tmp.data = "anger5"
        self.pub_face.publish(tmp)
        tmp.data = self.snd_anger[self.cnt_anger]
        self.pub_sound.publish(tmp)
        self.cnt_anger = self.get_next(self.cnt_anger, len(self.snd_anger))
        if self.mode == 1:
            tmp.data = "LR,0.4,2.0,0.6,0"
            self.pub_motion.publish(tmp)
    if face == 'Sadness':
        self.get_logger().info("\n###############\n### Sadness ###\n###############")
        tmp.data = "sadness5"
        self.pub_face.publish(tmp)
        tmp.data = self.snd_sadness[self.cnt_sadness]
        self.pub_sound.publish(tmp)
        self.cnt_sadness = self.get_next(self.cnt_sadness, len(self.snd_sadness))
        if self.mode == 1:
            tmp.data = "LR,0.2,2.0,0.2,0"
            self.pub_motion.publish(tmp)

def main(args=None):
  try:
    rclpy.init(args=args)
    talker = MotionCtrl()
    rclpy.spin(talker)
  except KeyboardInterrupt:
    pass
  finally:
    talker.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()
