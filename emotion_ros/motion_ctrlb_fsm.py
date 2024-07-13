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

class MotionCtrlFsm(Node):
  def __init__(self):
    super().__init__('motion_ctrlb_fsm')

    self.mode = 1 # motion switch
    self.old_face = '' # previous emotion

    ### FSM ###
    self.fsm_mode = 0
    self.action_node1_timer = -1
    self.yes_no = 0
    self.action_node2_timer = -1
    
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

  def action_node0(self, face):
    tmp = String()   
    if face == 'Happy':
        self.get_logger().info("\n#############\n### Happy ###\n#############")
        tmp.data = "happy5"
        self.pub_face.publish(tmp)
        tmp.data = self.snd_happy[self.cnt_happy]
        self.pub_sound.publish(tmp)
        cnt_happy = self.get_next(self.cnt_happy, len(self.snd_happy))
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
        cnt_relax = self.get_next(self.cnt_relax, len(self.snd_relax))
        if self.mode == 1:
            tmp.data = "LR,0.0,2.0,0.3,0"
            self.pub_motion.publish(tmp)
    if face == 'Anger':
        self.get_logger().info("\n#############\n### Anger ###\n#############")
        tmp.data = "anger5"
        self.pub_face.publish(tmp)
        tmp.data = self.snd_anger[self.cnt_anger]
        self.pub_sound.publish(tmp)
        ### Deep_B ###
        if self.snd_anger[self.cnt_anger] == "Deep_B.wav":
            self.fsm_mode = 1
        cnt_anger = self.get_next(self.cnt_anger, len(self.snd_anger))
        if self.mode == 1:
            tmp.data = "LR,0.4,2.0,0.6,0"
            self.pub_motion.publish(tmp)
    if face == 'Sadness':
        self.get_logger().info("\n###############\n### Sadness ###\n###############")
        tmp.data = "sadness5"
        self.pub_face.publish(tmp)
        tmp.data = self.snd_sadness[self.cnt_sadness]
        self.pub_sound.publish(tmp)
        ### Deep_B ###
        if self.snd_sadness[self.cnt_sadness] == "Deep_B.wav":
            self.fsm_mode = 1
        cnt_sadness = self.get_next(self.cnt_sadness, len(self.snd_sadness))
        if self.mode == 1:
            tmp.data = "LR,0.2,2.0,0.2,0"
            self.pub_motion.publish(tmp)

  def action_node1(self):
    if self.action_node1_timer < 0:
        self.action_node1_timer = 10
        ### Ask Yes / No ###
        
        ### debug motion ###
        tmp = String()
        tmp.data = "HA,0.4,2.0,0.6,0"
        self.pub_motion.publish(tmp)
    elif self.action_node1_timer == 0:
        self.action_node1_timer = -1
        ### Time up : Back to fsm_mode 0 ###
        self.fsm_mode = 0
    else:
        self.action_node1_timer = self.action_node1_timer - 1
    ### Confirm Yes / No ###
    if self.yes_no == 1:
        self.fsm_mode = 2
    return

  def action_node2(self):
    ### Deep breathing ###
    if self.action_node2_timer < 0:
        self.action_node2_timer = 10
        ### Start ###
        
        ### debug motion ###
        tmp = String()
        tmp.data = "HA,-0.4,2.0,0.6,0"
        self.pub_motion.publish(tmp)
    elif self.action_node2_timer == 0:
        self.action_node2_timer = -1
        ### Time up : Back to fsm_mode 0 ###
        self.fsm_mode = 0
    else:
        self.action_node2_timer = self.action_node2_timer - 1
    return

  def callback_emotion(self, msg):
    self.get_logger().info("Message " + str(msg.data) + " recieved")

    tmp = msg.data.split(',')
    face = tmp[0]
    EMO_PERCENTAGE = float(tmp[1])  # 感情強度（float）
    self.get_logger().info("face " + str(face) + ", EMO_PERCENTAGE " + str(EMO_PERCENTAGE))
    
    # Record the maximum intensity of each emotion
    '''
    if face == 'Happy':
        if EMO_PERCENTAGE > self.em_score[0]: self.em_score[0] = EMO_PERCENTAGE
    if face == 'Relax':
        if EMO_PERCENTAGE > self.em_score[1]: self.em_score[1] = EMO_PERCENTAGE
    if face == 'Anger':
        if EMO_PERCENTAGE > self.em_score[2]: self.em_score[2] = EMO_PERCENTAGE
    if face == 'Sadness':
        if EMO_PERCENTAGE > self.em_score[3]: self.em_score[3] = EMO_PERCENTAGE
    '''
    # Accumulate the intensity of each emotion
    if face == 'Happy':
        self.em_score[0] = self.em_score[0] + EMO_PERCENTAGE
    if face == 'Relax':
        self.em_score[1] = self.em_score[1] + EMO_PERCENTAGE
    if face == 'Anger':
        self.em_score[2] = self.em_score[2] + EMO_PERCENTAGE
    if face == 'Sadness':
        self.em_score[3] = self.em_score[3] + EMO_PERCENTAGE
    
    self.get_logger().info("em_score [" + str(self.em_score[0]) + "," + str(self.em_score[1]) + "," + str(self.em_score[2]) + "," + str(self.em_score[3]) + "]")

    do_action = 0
    self.recv_cnt = self.recv_cnt + 1
    if self.recv_cnt >= self.recv_max:
        self.recv_cnt = 0

        # Find the emotion that recorded maximum intensity
        max_em_score = 0
        max_em_id = 1  # Default : Relax
        for i in range(4):
            if self.em_score[i] > max_em_score:
                max_em_score = self.em_score[i]
                max_em_id = i
        self.em_score = [0, 0, 0, 0]

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

        do_action = 1
        if face != self.old_face:  # Respond when emotion changes
            self.rep_cnt = 0
        self.old_face = face
        if self.rep_cnt != 0:  # Respond once every rep_max While emotion continues
            self.rep_cnt = self.rep_cnt + 1
            if self.rep_cnt == self.rep_max: self.rep_cnt = 0
            do_action = 0
        else:
            self.rep_cnt = self.rep_cnt + 1
            if self.rep_cnt == self.rep_max: self.rep_cnt = 0

        # Edit a message for the emotion2 topic
        msg.data = face + "," + str(max_em_score)
        self.pub_emotion2.publish(msg)

    # Execute the actual response
    self.get_logger().info(">>> fsm_mode = " + str(self.fsm_mode))
    if self.fsm_mode == 0: # Question and answer
        if do_action == 1:
            self.action_node0(face)
        return
    if self.fsm_mode == 1: # Confirm deep breathing
        self.action_node1()
        return
    if self.fsm_mode == 2: # Deep breathing instruction
        self.action_node2()
        return

def main(args=None):
  try:
    rclpy.init(args=args)
    talker = MotionCtrlFsm()
    rclpy.spin(talker)
  except KeyboardInterrupt:
    pass
  finally:
    talker.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()
