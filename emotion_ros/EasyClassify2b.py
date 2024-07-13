#!/usr/bin/env python3
# -*- coding: utf-8 -*- 

import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import String

class EasyClassify2(Node):
  def __init__(self):
    super().__init__('EasyClassify2b')
    self.publisher = self.create_publisher(String, 'emotion', 10)
    self.create_subscription(String, 'bio_data', self.callback_bio_data, 10)
    self.create_subscription(String, 'hrveeg_data', self.callback_hrveeg, 10)
    self.THRESHOLD = 0.23

  def cal_arousal(self, msg):
    tmp = msg.data.split(',')
    arousal = float(tmp[0]) - float(tmp[1])
    self.get_logger().info("arousal: " + str(arousal))
    return arousal

  def cal_valence(self, msg):
    tmp = msg.data.split(',')
    pnn50 = float(tmp[4])
    if (pnn50 < self.THRESHOLD):
        valence = (100 - ((pnn50 / self.THRESHOLD) * 100)) * (-1)
    else:
        valence = ((pnn50 - self.THRESHOLD) / (1 - self.THRESHOLD)) * 100
    self.get_logger().info("valence: " + str(valence))
    return valence

  def cal_angle(self, arousal, valence):
    angle = math.atan2(arousal, valence) * 180 / math.pi
    if (angle < 0):
        angle += 360
    angle = int(angle)
    self.get_logger().info("Angle: " + str(angle))
    return angle

  def cal_distance(self, arousal, valence):
    distance = math.sqrt(arousal*arousal + valence*valence)
    self.get_logger().info("Distance: " + str(distance))
    return distance

  def classify_emotion(self, angle):
    if angle <= 90:
        return 'Happy'
    elif angle <= 180:
        return 'Anger'
    elif angle <= 270:
        return 'Sadness'
    else:
        return 'Relax'

  def estimate_emotion(self, msg):
    arousal = self.cal_arousal(msg)
    valence = self.cal_valence(msg)
    angle = self.cal_angle(arousal, valence)
    emo_value = self.cal_distance(arousal, valence)
    emo_name = self.classify_emotion(angle)
    return [emo_name, emo_value]

  def callback_bio_data(self, msg):
    self.THRESHOLD = float(msg.data)
    self.get_logger().info("pNN50th " + str(msg.data) + " recieved")

  def callback_hrveeg(self, msg):
    sub_emo = self.estimate_emotion(msg)
    pub_emo = String()
    pub_emo.data = str(sub_emo[0]) + "," + str(sub_emo[1])
    self.publisher.publish(pub_emo)
    self.get_logger().info("Emotion: " + str(sub_emo[0]) + ", Value: " + str(sub_emo[1]))

def main(args=None):
  try:
    rclpy.init(args=args)
    talker = EasyClassify2()
    rclpy.spin(talker)
  except KeyboardInterrupt:
    pass
  finally:
    talker.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()
