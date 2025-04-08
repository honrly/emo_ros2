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
import csv
from datetime import datetime
from zoneinfo import ZoneInfo
import threading

class SoundCtrl(Node):
  def __init__(self):
    super().__init__('sound_ctrl')
    
    self.volume = 100
    self.home_dir = os.environ["HOME"]
    
    #if rospy.has_param('~volume'):
    #    self.volume = int(rospy.get_param('~volume'))
    self.get_logger().info("Volume: " + str(self.volume))

    ### get playback ###
    fpath = self.home_dir + "/turtlebot3_ws/src/emotion_ros/Voice_JP/1secBrank.wav"
    audio = AudioSegment.from_file(fpath)
    audio_trim = audio[0:10]
    self.playback = _play_with_simpleaudio(audio_trim)

    self.create_subscription(String, 'speech', self.callback, 10)
    
    directory_path = '/home/user/turtlebot3_ws/src/emotion_ros'
      
    sound_data_path = os.path.join(directory_path, 'data_record/sound_data')
    os.makedirs(sound_data_path, exist_ok=True)

    timestamp = datetime.now(ZoneInfo("Asia/Tokyo")).strftime('%Y%m%d_%H%M%S')

    self.csv_filename = os.path.join(sound_data_path, f'{timestamp}_sound.csv')
    self.csv_file = open(self.csv_filename, mode='w', newline='')
    self.csv_writer = csv.writer(self.csv_file)
    self.csv_writer.writerow(['timestamp', 'sound'])

  def write_sound_data(self, sound):
    timestamp = datetime.now(ZoneInfo("Asia/Tokyo")).strftime('%H:%M:%S.%f')[:-3]
    self.csv_writer.writerow([timestamp, sound])
    self.csv_file.flush()
  
  def play_wav(self, sound_file, vol):
    fpath = self.home_dir + "/turtlebot3_ws/src/emotion_ros/Voice_JP/" + sound_file
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
    # self.get_logger().info("Message " + str(msg.data) + " recieved")
    self.write_sound_data(msg.data)
    self.play_wav(msg.data, self.volume)
    
  def ctrl_volume(self):
    while True:
      user_input = input("音量を変更（デフォルト100, 0~100）: ")
      try:
          volume = int(user_input)
          if 1 <= volume <= 100:
              self.volume = volume
              print(f"Volume: {self.volume}")
          else:
              print("1〜100の範囲で入力してください。")
      except ValueError:
          print("整数を入力してください。")

def main(args=None):
  try:
    rclpy.init(args=args)
    talker = SoundCtrl()
    volume_thread = threading.Thread(target=talker.ctrl_volume, daemon=True)
    volume_thread.start()
    rclpy.spin(talker)
  except KeyboardInterrupt:
    pass
  finally:
    talker.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()
