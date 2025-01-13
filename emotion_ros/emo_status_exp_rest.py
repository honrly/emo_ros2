import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, String
from my_custom_message.msg import PulseData, BrainData
import math
import numpy as np
import os
import csv
import time
from datetime import datetime
from zoneinfo import ZoneInfo

REST_TIME = 120
ROBOT_TIME = REST_TIME + 180
ALL_TIME = ROBOT_TIME + 120

class EmoStatusNode(Node):
    def __init__(self):
        super().__init__('emo_status_exp_rest')
        
        # Rest time manage
        self.pub_time_count = self.create_publisher(Int32, 'time_count', 10)
        self.time_count = Int32()
        self.time_count.data = 0
        
        self.stimu = 'None'
        
        self.create_subscription(PulseData, 'pulse', self.pulse_callback, 10)
        self.create_subscription(BrainData, 'brain_wave', self.brain_wave_callback, 10)
        
        # bio_data
        self.pulse = PulseData()
        self.brain = BrainData()
        
        self.beta_l_alpha_l = 0 # arousal

        self.pub_emo_status = self.create_publisher(String, 'emo_status', 10)
        self.emo_status = String() # 感情状態
        self.emo_status.data = '' # 送信データ
        
        self.THRESHOLD_VALENCE = -1.0 # valenceの平均指標
        self.THRESHOLD_AROUSAL = -1.0 # arousalの平均指標
        
        self.FLAG_THRESHOLD = 0
        
        self.pnn20_rest = []
        self.pnn50_rest = []
        self.beta_l_alpha_l_rest = []

        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.publish_emo_status)
    
        # Record csv
        directory_path = '/home/user'

        bio_pulse_data_path = os.path.join(directory_path, 'data_record/bio_data/pulse')
        bio_brain_data_path = os.path.join(directory_path, 'data_record/bio_data/brain')
        emo_data_path = os.path.join(directory_path, 'data_record/emo_data')

        os.makedirs(bio_pulse_data_path, exist_ok=True)
        os.makedirs(bio_brain_data_path, exist_ok=True)
        os.makedirs(emo_data_path, exist_ok=True)

        timestamp = datetime.now(ZoneInfo("Asia/Tokyo")).strftime('%Y%m%d_%H%M%S')

        self.pulse_csv_filename = os.path.join(bio_pulse_data_path, f'{timestamp}_rest.csv')
        self.pulse_csv_file = open(self.pulse_csv_filename, mode='w', newline='')
        self.pulse_csv_writer = csv.writer(self.pulse_csv_file)
        self.pulse_csv_writer.writerow(['timestamp', 'threshold_pnn', 'bpm', 'ibi', 'sdnn', 'cvnn', 'rmssd', 'pnn10', 'pnn20', 'pnn30', 'pnn40', 'pnn50'])
        
        self.brain_csv_filename = os.path.join(bio_brain_data_path, f'{timestamp}_rest.csv')
        self.brain_csv_file = open(self.brain_csv_filename, mode='w', newline='')
        self.brain_csv_writer = csv.writer(self.brain_csv_file)
        self.brain_csv_writer.writerow(['timestamp', 'threshold_b_a', 'beta_l_alpha_l', 'poorsignal', 'delta', 'theta', 
                                        'alpha_l', 'alpha_h', 'beta_l', 'beta_h', 'gamma_l', 'gamma_m'])

        self.emo_csv_filename = os.path.join(emo_data_path, f'{timestamp}_rest.csv')
        self.emo_csv_file = open(self.emo_csv_filename, mode='w', newline='')
        self.emo_csv_writer = csv.writer(self.emo_csv_file)
        self.emo_csv_writer.writerow(['timestamp', 'stimu', 'emo', 'threshold_b_a', 'beta_l_alpha_l', 'poorsignal', 'delta', 'theta', 
                                      'alpha_l', 'alpha_h', 'beta_l', 'beta_h', 'gamma_l', 'gamma_m', 'threshold_pnn', 'bpm', 'ibi', 
                                      'sdnn', 'cvnn', 'rmssd', 'pnn10', 'pnn20', 'pnn30', 'pnn40', 'pnn50'])
    
    def write_bio_pulse_data(self):
        timestamp = datetime.now(ZoneInfo("Asia/Tokyo")).strftime('%H:%M:%S.%f')[:-3]
        self.pulse_csv_writer.writerow([timestamp, self.THRESHOLD_VALENCE, self.pulse._bpm, self.pulse._ibi, self.pulse._sdnn, self.pulse._cvnn, self.pulse._rmssd, 
                                        self.pulse._pnn10, self.pulse._pnn20, self.pulse._pnn30, self.pulse._pnn40, self.pulse._pnn50])
        self.pulse_csv_file.flush()
        
    def write_bio_brain_data(self):
        timestamp = datetime.now(ZoneInfo("Asia/Tokyo")).strftime('%H:%M:%S.%f')[:-3]
        self.brain_csv_writer.writerow([timestamp, self.THRESHOLD_AROUSAL, self.beta_l_alpha_l, self.brain._poorsignal, self.brain._delta, self.brain._theta, 
                                        self.brain._alpha_l, self.brain._alpha_h, self.brain._beta_l, self.brain._beta_h, self.brain._gamma_l, self.brain._gamma_m])
        self.brain_csv_file.flush()
    
    def write_emo_data(self, emo):
        timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        self.emo_csv_writer.writerow([timestamp, self.stimu, emo, self.THRESHOLD_AROUSAL, self.beta_l_alpha_l, self.brain._poorsignal, self.brain._delta, self.brain._theta, 
                                      self.brain._alpha_l, self.brain._alpha_h, self.brain._beta_l, self.brain._beta_h, self.brain._gamma_l, self.brain._gamma_m, 
                                      self.THRESHOLD_VALENCE, self.pulse._bpm, self.pulse._ibi, self.pulse._sdnn, self.pulse._cvnn, self.pulse._rmssd, 
                                      self.pulse._pnn10, self.pulse._pnn20, self.pulse._pnn30, self.pulse._pnn40, self.pulse._pnn50])
        self.emo_csv_file.flush()
        
    def pulse_callback(self, msg):
        self.pulse._bpm = msg.bpm
        self.pulse._ibi = msg.ibi
        self.pulse._sdnn = msg.sdnn
        self.pulse._cvnn = msg.cvnn
        self.pulse._rmssd = msg.rmssd
        self.pulse._pnn10 = msg.pnn10
        self.pulse._pnn20 = msg.pnn20
        self.pulse._pnn30 = msg.pnn30
        self.pulse._pnn40 = msg.pnn40
        self.pulse._pnn50 = msg.pnn50
        
        if self.FLAG_THRESHOLD == 0:
            self.pnn20_rest.append(self.pulse._pnn20)
            self.pnn50_rest.append(self.pulse._pnn50)
        
        self.write_bio_pulse_data()
        self.get_logger().info(f'PNN50: {self.pulse._pnn50}')
    
    def brain_wave_callback(self, msg):        
        self.brain._poorsignal = msg.poorsignal
        self.brain._delta = msg.delta
        self.brain._theta = msg.theta
        self.brain._alpha_l = msg.alpha_l
        self.brain._alpha_h = msg.alpha_h
        self.brain._beta_l = msg.beta_l
        self.brain._beta_h = msg.beta_h
        self.brain._gamma_l = msg.gamma_l
        self.brain._gamma_m = msg.gamma_m
        
        if msg.alpha_l != 0 and msg.beta_l != 0:
            self.beta_l_alpha_l = msg.beta_l / msg.alpha_l
        
        if self.FLAG_THRESHOLD == 0:
            self.beta_l_alpha_l_rest.append(self.beta_l_alpha_l) 
        
        self.write_bio_brain_data()        
        self.get_logger().info(f'LOWBETA / LOWALPHA {self.beta_l_alpha_l}, {msg.beta_l}, {msg.alpha_l}')
    
    def estimate_emotion_rest_base(self):
        emo_name = ''
        
        if self.FLAG_THRESHOLD != 0:
            # Happy
            if self.beta_l_alpha_l >= self.THRESHOLD_AROUSAL and self.pulse._pnn20 >= self.THRESHOLD_VALENCE:
                emo_name = 'Happy'   
            # Relax
            elif self.beta_l_alpha_l < self.THRESHOLD_AROUSAL and self.pulse._pnn20 >= self.THRESHOLD_VALENCE:
                emo_name = 'Relax'
            # Sadness
            elif self.beta_l_alpha_l < self.THRESHOLD_AROUSAL and self.pulse._pnn20 < self.THRESHOLD_VALENCE:
                emo_name = 'Sadness'
            # Anger
            elif self.beta_l_alpha_l >= self.THRESHOLD_AROUSAL and self.pulse._pnn20 < self.THRESHOLD_VALENCE:
                emo_name = 'Anger'

            self.emo_status.data = emo_name
    
    def publish_emo_status(self):
        if self.time_count.data < REST_TIME:
            self.stimu = 'Rest1'
            self.emo_status.data = 'Rest1'
        
        elif self.time_count.data >= REST_TIME and self.time_count.data < ROBOT_TIME:
            self.stimu = 'Robot'
            
            if self.FLAG_THRESHOLD == 0:
                self.THRESHOLD_AROUSAL = np.mean(self.beta_l_alpha_l_rest)
                self.THRESHOLD_VALENCE = np.mean(self.pnn20_rest)
                self.FLAG_THRESHOLD = 1
            
            self.estimate_emotion_rest_base()
            
        elif self.time_count.data >= ROBOT_TIME and self.time_count.data < ALL_TIME:
            self.stimu = 'Rest2'
            self.emo_status.data = 'Rest2'
        
        elif self.time_count.data >= ALL_TIME:
            while True:
                self.get_logger().info(f"time_count: {self.time_count.data}, EXP Finish")
                time.sleep(1)
        
        self.pub_emo_status.publish(self.emo_status)
        
        self.time_count.data += 1
        self.get_logger().info(f'time_count {self.time_count.data}')
        self.pub_time_count.publish(self.time_count)
        
        self.write_emo_data(self.emo_status.data)  
        
        self.get_logger().info(f'Stimu: {self.stimu}, Emotion: {self.emo_status.data}, lowb/a: {self.beta_l_alpha_l}, pnn20: {self.pulse._pnn20}')
        self.get_logger().info(f'THRESHOLD_VALENCE, {self.THRESHOLD_VALENCE}')
        self.get_logger().info(f'THRESHOLD_AROUSAL, {self.THRESHOLD_AROUSAL}\n\n')
    
    def run(self):
        while rclpy.ok():
            rclpy.spin(self)
            
def main(args=None):
    rclpy.init(args=args)
    talker = EmoStatusNode()
    try:
        talker.run()
    except KeyboardInterrupt:
        pass
    finally:
        talker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
