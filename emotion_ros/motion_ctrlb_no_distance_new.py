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
    super().__init__('motion_ctrlb_no_distance')
    
    self.em_score2 = []
    
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

  def sort_face_list(self):
    emotions = {0: 'Happy', 1: 'Relax', 2: 'Anger', 3: 'Sadness'}

    indexed_values = [(value, index) for index, value in enumerate(self.em_score)]

    indexed_values.sort(reverse=True, key=lambda x: x[0])

    new_list = []
    for value, index in indexed_values:
        new_list.extend([index] * value)

    # emotion_sort_list = [emotions[num] for num in new_list]
    return new_list

  
  def callback_emotion(self, msg):
    face = msg.data
    self.get_logger().info("Message " + str(msg.data) + " recieved")
    
    tmp = String()
    if face in ('Rest1', 'Rest2', 'Stimu1', 'Stimu2'):
        self.get_logger().info("\n###############\n### Rest ###\n###############")
        tmp.data = 'initial'
        self.pub_face.publish(tmp)
        return
    
    # 感情毎の強度を累積
    if face == 'Happy':
        self.em_score[0] += 1
    if face == 'Relax':
        self.em_score[1] += 1
    if face == 'Anger':
        self.em_score[2] += 1
    if face == 'Sadness':
        self.em_score[3] += 1
    
    self.get_logger().info("em_score [" + str(self.em_score[0]) + "," + str(self.em_score[1]) + "," + str(self.em_score[2]) + "," + str(self.em_score[3]) + "]")

    if len(self.em_score2) == 0:
        self.get_logger().info(f'no_em_score_len: {len(self.em_score2)}')
    else:
        face1 = String()
        em_id = self.em_score2.pop(0)
        if em_id == 0:
            face1.data = 'happy5'
        if em_id == 1:
            face1.data = 'relax5'
        if em_id == 2:
            face1.data = 'anger5'
        if em_id == 3:
            face1.data = 'sadness5'
        
        self.pub_face.publish(face1)
        self.get_logger().info(f'em_score_len: {len(self.em_score2)}')

    
    self.recv_cnt = self.recv_cnt + 1
    if self.recv_cnt < self.recv_max:
        return
    self.recv_cnt = 0

    # 最大強度を記録した感情を求める
    max_em_score = 0
    max_em_id = 1  # Default : Relax
    
    self.em_score2 = self.sort_face_list()
    self.get_logger().info(f'em_score_len: {len(self.em_score2)}')
    
    for i in range(4):
        if self.em_score[i] > max_em_score:
            max_em_score = self.em_score[i]
            max_em_id = i
    self.em_score = [0, 0, 0, 0]
    
    max_em_id = self.em_score2.pop(0)
    self.get_logger().info(f'em_score_len: {len(self.em_score2)}')

    if max_em_id == 0:
        face = 'Happy'
        self.get_logger().info("\n#############\n### Happy ###\n#############")
    if max_em_id == 1:
        face = 'Relax'
        self.get_logger().info("\n#############\n### Relax ###\n#############")
    if max_em_id == 2:
        face = 'Anger'
        self.get_logger().info("\n#############\n### Anger ###\n#############")
    if max_em_id == 3:
        face = 'Sadness'
        self.get_logger().info("\n###############\n### Sadness ###\n###############")
    
    
    #if face == self.old_face: return    # 感情が変化したときだけ、応答を変える
    if face != self.old_face:  # 感情状態が変化したときは応答する
        self.rep_cnt = 0
    self.old_face = face
    if self.rep_cnt != 0:  # 感情状態が継続中はrep_max回に1回応答する
        self.rep_cnt = self.rep_cnt + 1
        if self.rep_cnt == self.rep_max: self.rep_cnt = 0
        return
    else:
        self.rep_cnt = self.rep_cnt + 1
        if self.rep_cnt == self.rep_max: self.rep_cnt = 0
    
    # emotion2トピック用に最大強度を記録した感情と強度をメッセージに編集
    # Edit the emotion and intensity recorded with maximum intensity into a message for the emotion2 topic
    msg.data = face + "," + str(max_em_score)
    self.pub_emotion2.publish(msg)

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
