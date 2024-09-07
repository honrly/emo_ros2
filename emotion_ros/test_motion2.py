#!/usr/bin/env python3
# -*- coding: utf-8 -*- 

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf_transformations
from tf_transformations import euler_from_quaternion
import json
import sys
import numpy as np
import math
from sensor_msgs.msg import LaserScan   #message from laser range finder
import os
import csv
from datetime import datetime
from zoneinfo import ZoneInfo

#########################
# MQTT
#########################
import paho.mqtt.client as mqtt     # Import MQTT
import paho.mqtt.publish as publish
import time

class MyMQTTClass(mqtt.Client):
  def __init__(self):
    super(MyMQTTClass, self).__init__(mqtt.CallbackAPIVersion.VERSION1)
    self.recieve_data = ""
    self.recieve_time = ""
    self.lasttime     = ""

  def on_connect(self, mqttc, obj, flags, rc):
    print("rc: "+str(rc))
    sys.stdout.flush()

  def on_message(self, mqttc, obj, msg):
    print(msg.topic+" "+str(msg.qos)+" "+str(msg.payload))
    sys.stdout.flush()
    self.recieve_time = time.time()
    self.recieve_data = (msg.payload).decode()

  def run(self, hostname, topic):
    self.connect(hostname, 1883, 60)
    self.subscribe(topic, 0)

    self.loop_start()
    rc = 0
    return rc

  def publish_message(self, host_name, topic, message):
    publish.single(topic, message, hostname=host_name)

  def isNew(self):
    flag = False
    if self.lasttime==self.recieve_time:
      flag =  False
    else:
      flag = True
      self.lasttime = self.recieve_time
      #print("isNew : {}".format(flag))
    #self.lasttime = self.recieve_time
    return flag

mqttc_a = MyMQTTClass()

# Callback when getting messages
def mqttcallback_a():
  global mqttc_a
  try:
    recievedata =  str(mqttc_a.recieve_data)
    json_str = json.loads(recievedata)
    #print("json_str: {}".format(json_str))
    #sys.stdout.flush()
    #print("json_str[size]: {}".format(json_str["size"]))
    #sys.stdout.flush()
    n = (json_str["size"])
    #print("n: {}".format(n))
    #sys.stdout.flush()
    if n == 1:
      '''
      v1_str = json_str["values"]
      print("v1_str: {}".format(v1_str))
      sys.stdout.flush()
      v2_str = v1_str[0]["values"]
      print("v2_str: {}".format(v2_str))
      sys.stdout.flush()
      xyz = v2_str["x"]
      '''
      xyz = json_str["values"][0]["values"]["x"]
      print("xyz: {}".format(xyz))
      sys.stdout.flush()
      x = float(xyz[0])
      y = float(xyz[1])
      z = float(xyz[2])
      #a = np.arctan2(-y, -x)
      a = np.arctan2(y, x)
      print(">>>>>> x,y,z,a: {},{},{},{}".format(x, y, z, a))
      sys.stdout.flush()
      return(True, a)
  except BaseException as ex:
    print(ex)
    sys.stdout.flush()
  return(False, 0.0)

mqttc_a.run("localhost", "HARK_result")
#########################

class TestMotion(Node):
  def __init__(self):
    super().__init__('test_motion')
    
    self._odom_x = 0.0
    self._odom_y = 0.0
    self._odom_theta = 0.0
    self.dst_a = 0.0
    self.kspd = 0.2
    self.tspd = 0.6
    self.rept = 0
    self.motion_mode = 0
    self.org_a = 0.0
    self.closest_no = 0
    self.closest_range = 0.0
    self.closest_dir = 0.0
    
    self.create_subscription(Odometry, 'odom', self.callback_odom, 10)
    self.create_subscription(LaserScan, 'scan', self.callback_scan, 10)
    self.create_subscription(String, 'motion', self.callback_motion, 10)
    self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
    timer_period = 0.05 # sec
    self.timer = self.create_timer(timer_period, self.pub_callback_timer)
    
    # Record csv
    # directory_path = '/home/user/ros2_ws/src/emotion_ros'
    directory_path = '/home/user/turtlebot3_ws/src/emotion_ros'
      
    motion_data_path = os.path.join(directory_path, 'data_record/motion_data')
    os.makedirs(motion_data_path, exist_ok=True)

    timestamp = datetime.now(ZoneInfo("Asia/Tokyo")).strftime('%Y%m%d_%H%M%S')

    self.csv_filename = os.path.join(motion_data_path, f'{timestamp}_fix_num.csv')
    self.csv_file = open(self.csv_filename, mode='w', newline='')
    self.csv_writer = csv.writer(self.csv_file)
    self.csv_writer.writerow(['timestamp', 'motion'])

  def write_motion_data(self, motion):
    timestamp = datetime.now(ZoneInfo("Asia/Tokyo")).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
    self.csv_writer.writerow([timestamp, motion])
    self.csv_file.flush()

  def pub_callback_timer(self):
    global mqttc_a
    if mqttc_a.isNew(): 
        flg, a = mqttcallback_a()
        if flg == True:
          self.dst_a = a
          self.kspd = 0.2
          self.tspd = 0.6
          self.motion_mode = 1
    if self.motion_mode == 1: # head angle move
      self.move_angle(self.kspd, self.tspd)
    elif self.motion_mode == 2: # LR
      self.move_angle(self.kspd, self.tspd)
      if self.motion_mode == 0: # Goal
        self.dst_a = -self.dst_a
        self.motion_mode = 2

  def stop_motion(self):
    twist = Twist()
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    #twist.angular.z = 0.0
    twist.angular.z = self.org_a
    self.pub_cmd_vel.publish(twist)
    #self.get_logger().info("Stop: {}".format("+++"))

  def calc_dif_angle(self):
    pi = 3.14159
    #dst = self.dst_a
    dst = self.dst_a + self.org_a
    odm = self._odom_theta
    #print("dst={}, odm={}".format(dst, odm))
    #sys.stdout.flush()
    if odm < -pi:
      odm += pi
    elif odm > pi:
      odm -= pi
    dif = dst - odm
    return dif
    
  def move_angle(self, kspd, tspd): 
    if self.motion_mode == 0: return
    #dif = self.dst_a - self._odom_theta
    dif = self.calc_dif_angle()
    th = 0.01#0.5#0.1#
    #self.get_logger().info("move_angle: dif " + str(dif))
    if dif < th and dif > -th:
    #if abs(dif) < th:
      self.motion_mode = 0
      self.stop_motion()
      return
    twist = Twist()
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = np.sign(dif) * min(abs(dif * kspd), tspd)
    self.pub_cmd_vel.publish(twist)
    #self.get_logger().info("Publishing: {}".format("+++"))

  def callback_odom(self, msg):
    self._odom_x = msg.pose.pose.position.x
    self._odom_y = msg.pose.pose.position.y
    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w
    q = (qx, qy, qz, qw)
    e = euler_from_quaternion(q)
    self._odom_theta = e[2] 
    #self.get_logger().info("\n<<< Odomery: x=" + str(_odom_x) + " y=" + str(_odom_y) + " theta=" + str(_odom_theta) + " >>>")
    #self.get_logger().info("   <<< theta=" + str(_odom_theta))

  def callback_scan(self, msg):
    delta_a = (msg.angle_max - msg.angle_min) / len(msg.ranges)
    thresh_a = 45.0 / 180.0 * math.pi
    odom_a = self._odom_theta
    min_no = -1
    min_rng = 1000.0
    min_dir = 0.0
    for i in range(len(msg.ranges)):
        rng = msg.ranges[i]
        if rng < msg.range_min: continue
        if rng > msg.range_max: continue
        if rng < min_rng:
            dir = msg.angle_min + (i * delta_a)
            if dir > math.pi:
                dir = dir - (math.pi * 2.0)
            elif dir < -math.pi:
                dir = dir + (math.pi * 2.0)
            if dir >= -thresh_a-odom_a and dir <= thresh_a-odom_a:
                min_no = i
                min_rng = rng
                min_dir = dir
    if min_no >= 0:
        self.closest_no = min_no
        self.closest_range = min_rng
        self.closest_dir = min_dir + odom_a
    #self.get_logger().info("\n### Scan: no=" + str(self.closest_no) + " range=" + str(self.closest_range) + " dir=" + str(self.closest_dir) + " ###")
    self.org_a = self.closest_dir

  def callback_motion(self, msg):
    self.write_motion_data(msg.data)
    self.get_logger().info("Message " +  str(msg.data) + " recieved")
    tmp = str(msg.data).split(",")
    self.get_logger().info("Message " + tmp[0] + ", " + tmp[1] + ", " + tmp[2] + ", " + tmp[3] + ", " + tmp[4])
    if tmp[0] == "HA":
      self.dst_a = float(tmp[1])
      self.kspd = float(tmp[2])
      self.tspd = float(tmp[3])
      self.rpet = int(tmp[4])
      self.motion_mode = 1
    if tmp[0] == "LR":
      self.dst_a = float(tmp[1])
      self.kspd = float(tmp[2])
      self.tspd = float(tmp[3])
      self.rpet = int(tmp[4])
      self.motion_mode = 2

def main(args=None):
  try:
    rclpy.init(args=args)
    talker = TestMotion()
    rclpy.spin(talker)
  except KeyboardInterrupt:
    pass
  finally:
    talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()
