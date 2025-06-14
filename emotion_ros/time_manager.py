import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from my_custom_message.msg import BrainData
from time import sleep
import datetime
import sys
import time
import os
import psutil
import filecmp
import threading
import cv2

REST = 180
ROBOT = 300

home_dir = os.environ["HOME"]
os.system("setterm -cursor off>/dev/tty0")

def disp(img_file):
    command1 = "cat " + home_dir + "/turtlebot3_ws/src/emotion_ros/face_DB/" + img_file + ".raw > /dev/fb0"
    command2 = "cat /dev/fb0 > /home/ubuntu/tmp.raw"
    os.system(command1)


class TimeManagerNode(Node):
    def __init__(self):
        super().__init__('time_manager_node')
        
        disp("initial")

        # MindWave Setting
        self.mw = MindWave(address=MINDWAVE_ADDRESS, autostart=False, verbose=3)
        # self.mw = MindWave(autostart=False, verbose=3)  # Autoscan for MindWave Mobile
        self.mw.set_callback('signal', self.publish_brain_wave) # poor signal
        self.mw.set_callback('eeg', self.set_brain_data)
        self.mw.start()

        # Publish MindWave Data
        self.pub_brain_wave = self.create_publisher(BrainData, 'brain_wave', 10)
        self.brain_wave = BrainData()
        self.brain_wave.poorsignal = 0
        self.brain_wave.delta = 0
        self.brain_wave.theta = 0
        self.brain_wave.alpha_l = 0
        self.brain_wave.alpha_h= 0
        self.brain_wave.beta_l = 0
        self.brain_wave.beta_h = 0
        self.brain_wave.gamma_l = 0
        self.brain_wave.gamma_m = 0

        #timer_period = 1.0
        #self.timer = self.create_timer(timer_period, self.publish_)

    def set_brain_data(self, eeg_data):
        self.brain_wave.delta = eeg_data['delta']
        self.brain_wave.theta = eeg_data['theta']
        self.brain_wave.alpha_l = eeg_data['alpha_l']
        self.brain_wave.alpha_h= eeg_data['alpha_h']
        self.brain_wave.beta_l = eeg_data['beta_l']
        self.brain_wave.beta_h = eeg_data['beta_h']
        self.brain_wave.gamma_l = eeg_data['gamma_l']
        self.brain_wave.gamma_m = eeg_data['gamma_m']
        
        self.get_logger().info(f'EED_DATA: {eeg_data}\n\n')

    def publish_brain_wave(self, poorsignal_data):
        self.brain_wave.poorsignal = poorsignal_data
        self.get_logger().info(f'POORSIGNAL: {self.brain_wave.poorsignal}')
        self.pub_brain_wave.publish(self.brain_wave)
    
    def run(self):
        sleep(REST)
        disp("happy1")
        pubrobot
        sleep(ROBOT)
        pubend
        playowarionsei
        disp("initial")
        sleep(REST)
            

def main(args=None):
    rclpy.init(args=args)
    talker = TimeManagerNode()
    try:
        talker.run()
    except KeyboardInterrupt:
        pass
    except TimeoutError:
        pass
    finally:
        talker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
