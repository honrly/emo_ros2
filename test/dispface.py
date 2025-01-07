#!/usr/bin/env python3
# -*- coding: utf-8 -*- 
import sys
import time
import os
import psutil
import filecmp
import threading
import cv2

home_dir = os.environ["HOME"]

#disp("initial")
os.system("setterm -cursor off>/dev/tty0")

def cmp_file(fn1, fn2):
    f1 = open(fn1, 'rb')
    f2 = open(fn2, 'rb')

    string1 = f1.read()
    string2 = f2.read()

    if string1 == string2: return True
    return False

### 採用版 ###
def disp(img_file):
    command1 = "cat " + home_dir + "/turtlebot3_ws/src/emotion_ros/face_DB/" + img_file + ".raw > /dev/fb0"
    command2 = "cat /dev/fb0 > /home/ubuntu/tmp.raw"
    os.system(command1)
   
def disp_cv(img_file):
    fff = home_dir + "/turtlebot3_ws/src/emotion_ros/face_DB/" + img_file + ".JPG"
    img = cv2.imread(fff)
    img_resize = cv2.resize(img, (320,180))
    cv2.imshow("Face", img_resize)
    cv2.waitKey(1)
    #.get_logger().info("\n###\n### Disp " + str(fff) + "\n###")

def disp_thread(target_image):
    disp(target_image)
    #.disp_cv(target_image)
    time.sleep(0.1)

def main():
  try:
    disp("BLACK")

    #t = threading.Thread(args=(), target=disp_thread)
    #t.start()
    disp("happy5")
  except KeyboardInterrupt:
    pass
  finally:
    return
    #os.system("setterm -cursor on>/dev/tty0")
  
if __name__ == '__main__':
    main()
