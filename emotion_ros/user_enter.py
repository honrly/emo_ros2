#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import sys
import csv
from datetime import datetime
import os
from pynput import keyboard

class UserEnter(Node):
    def __init__(self):
        super().__init__('user_enter')

        directory_path = '/home/user/ros2_ws/src/emotion_ros'
        enter_data_path = os.path.join(directory_path, 'bio_record/enter_data')
        os.makedirs(enter_data_path, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_filename = os.path.join(enter_data_path, f'{timestamp}_enter.csv')
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'NoRobot', 'NoChangeMy', 'NoChangeRobot'])

        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def on_press(self, key):
        NoRobot = 0
        NoChangeMy = 0
        NoChangeRobot = 0
        
        if key == keyboard.KeyCode.from_char('1'):
            NoRobot = 1
            current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            self.get_logger().info(f'1 key pressed at {current_time}')
            self.csv_writer.writerow([current_time, NoRobot, NoChangeMy, NoChangeRobot])
            self.csv_file.flush()
            NoRobot = 0
        
        if key == keyboard.KeyCode.from_char('2'):
            NoChangeMy = 1
            current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            self.get_logger().info(f'2 key pressed at {current_time}')
            self.csv_writer.writerow([current_time, NoRobot, NoChangeMy, NoChangeRobot])
            self.csv_file.flush()
            NoChangeMy = 0
            
        if key == keyboard.KeyCode.from_char('3'):
            NoChangeRobot = 1
            current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            self.get_logger().info(f'3 key pressed at {current_time}')
            self.csv_writer.writerow([current_time, NoRobot, NoChangeMy, NoChangeRobot])
            self.csv_file.flush()
            NoChangeRobot = 0

    def run(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    node = UserEnter()
    try:
        node.run()
    except KeyboardInterrupt:
        current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        print(f'FINAL {current_time}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()