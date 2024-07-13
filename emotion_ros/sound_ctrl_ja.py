#!/usr/bin/env python3
# -*- coding: utf-8 -*- 

import rclpy
from rclpy.node import Node
import random
import math
from pydub import AudioSegment
from pydub.playback import play
from pydub.playback import _play_with_simpleaudio
from std_msgs.msg import String
import os

class SoundCtrl(Node):
  def __init__(self):
    super().__init__('sound_ctrl')
    
    self.volume = 100
    self.home_dir = os.environ["HOME"]
    
    #if rospy.has_param('~volume'):
    #    self.volume = int(rospy.get_param('~volume'))
    self.get_logger().info("Volume: " + str(self.volume))

    ### get playback ###
    fpath = self.home_dir + "/turtlebot3_ws/src/emotion_ros/Voice_EN/1secBrank.wav"
    audio = AudioSegment.from_file(fpath)
    audio_trim = audio[0:10]
    self.playback = _play_with_simpleaudio(audio_trim)

    self.create_subscription(String, 'speech', self.callback, 10)

  def play_wav(self, sound_file, vol):
    fpath = self.home_dir + "/turtlebot3_ws/src/emotion_ros/Voice_JA/" + sound_file
    audio = AudioSegment.from_file(fpath)
    if self.playback.is_playing():
        self.playback.stop()
    if vol != 100:
        audio_mod = audio + (20 * math.log10(vol/100))
        #play(audio_mod)
        self.playback = _play_with_simpleaudio(audio_mod)
    else:
        #play(audio)
        self.playback = _play_with_simpleaudio(audio)

  def callback(self, msg):
    self.get_logger().info("Message " + str(msg.data) + " recieved")
    self.play_wav(msg.data, self.volume)

def main(args=None):
  try:
    rclpy.init(args=args)
    talker = SoundCtrl()
    rclpy.spin(talker)
  except KeyboardInterrupt:
    pass
  finally:
    talker.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()
