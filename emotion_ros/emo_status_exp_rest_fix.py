import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, String
from my_custom_message.msg import PulseData, BrainData
import math
import os
import csv
from datetime import datetime

class EmoStatusNode(Node):
    def __init__(self):
        super().__init__('emo_status_exp_rest_fix')
        
        # Rest time manage
        self.REST_TIME = 30
        self.pub_time_count = self.create_publisher(Int32, 'time_count', 10)
        self.time_count = Int32()
        self.time_count.data = 0
        
        self.stimu = 'None'
        
        self.create_subscription(PulseData, 'pulse', self.pulse_callback, 10)
        self.create_subscription(BrainData, 'brain_wave', self.brain_wave_callback, 10)
        
        # bio_data
        self.bpm = 0
        self.ibi = 0
        self.sdnn = 0
        self.cvnn = 0
        self.rmssd = 0
        self.pnn10 = 0.0 # valence
        self.pnn20 = 0.0
        self.pnn30 = 0.0
        self.pnn40 = 0.0
        self.pnn50 = 0.0
        self.rmssd = 0.0
        
        self.poorsignal = 0
        self.delta = 0
        self.theta = 0
        self.alpha_l = 0
        self.alpha_h = 0
        self.beta_l = 0
        self.beta_h = 0
        self.gamma_l = 0
        self.gamma_m = 0
        
        self.beta_l_alpha_l = 0 # arousal

        self.pub_emo_status = self.create_publisher(String, 'emo_status', 10)
        self.emo_status = String() # 感情状態
        self.emo_status.data = '' # 送信データ
        
        self.THRESHOLD_VALENCE = -1.0 # valenceの平均指標
        self.THRESHOLD_AROUSAL = -1.0 # arousalの平均指標
        self.THRESHOLD_VALENCE_fix = 0.236 # valenceの平均指標
        self.THRESHOLD_AROUSAL_fix = 1.0 # arousalの平均指標
        
        self.FLAG_THRESHOLD = 0
        
        self.pnn10_rest = []
        self.pnn20_rest = []
        self.pnn30_rest = []
        self.pnn40_rest = []
        self.pnn50_rest = []
        self.rmssd_rest = []
        self.beta_l_alpha_l_rest = []

        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.publish_emo_status)
    
        # Record csv
        # directory_path = '/home/user/ros2_ws/src/emotion_ros'
        directory_path = '/home/user/turtlebot3_ws/src/emotion_ros'
         
        bio_data_path = os.path.join(directory_path, 'bio_record/bio_data')
        emo_data_path = os.path.join(directory_path, 'bio_record/emo_data')

        os.makedirs(bio_data_path, exist_ok=True)
        os.makedirs(emo_data_path, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        self.csv_filename = os.path.join(bio_data_path, f'{timestamp}_rest_base.csv')
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'threshold_b_a', 'threshold_pnn', 'beta_l_alpha_l', 
                                'poorsignal', 'delta', 'theta', 'alpha_l', 'alpha_h', 'beta_l', 'beta_h', 'gamma_l', 'gamma_m', 
                                'bpm', 'ibi', 'sdnn', 'cvnn', 'rmssd', 'pnn10', 'pnn20', 'pnn30', 'pnn40', 'pnn50'])

        self.emo_csv_filename = os.path.join(emo_data_path, f'{timestamp}_rest_base.csv')
        self.emo_csv_file = open(self.emo_csv_filename, mode='w', newline='')
        self.emo_csv_writer = csv.writer(self.emo_csv_file)
        self.emo_csv_writer.writerow(['timestamp', 'stimu', 'emo', 'beta_l_alpha_l', 'pnn10', 'pnn20', 'pnn30', 'pnn40', 'pnn50', 'threshold_b_a', 'threshold_pnn'])

    
    def write_bio_data(self):
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        self.csv_writer.writerow([timestamp, self.THRESHOLD_AROUSAL, self.THRESHOLD_VALENCE, self.beta_l_alpha_l, 
                                  self.poorsignal, self.delta, self.theta, self.alpha_l, self.alpha_h, self.beta_l, self.beta_h, self.gamma_l, self.gamma_m, 
                                  self.bpm, self.ibi, self.sdnn, self.cvnn, self.rmssd, self.pnn10, self.pnn20, self.pnn30, self.pnn40, self.pnn50])
        self.csv_file.flush()
        
    def pulse_callback(self, msg):
        self.pnn10 = msg.pnn10
        self.pnn20 = msg.pnn20
        self.pnn30 = msg.pnn30
        self.pnn40 = msg.pnn40
        self.pnn50 = msg.pnn50
        self.rmssd = msg.rmssd
        self.get_logger().info(f'PNN50: {self.pnn50}')
        
        self.write_bio_data()
    
    def brain_wave_callback(self, msg):        
        self.poorsignal = msg.poorsignal
        self.delta = msg.delta
        self.theta = msg.theta
        self.alpha_l = msg.alpha_l
        self.alpha_h = msg.alpha_h
        self.beta_l = msg.beta_l
        self.beta_h = msg.beta_h
        self.gamma_l = msg.gamma_l
        self.gamma_m = msg.gamma_m
        
        self.beta_l_alpha_l = msg.beta_l / msg.alpha_l
        
        self.get_logger().info(f'LOWBETA / LOWALPHA {self.beta_l_alpha_l}, {msg.beta_l}, {msg.alpha_l}')
        
        self.write_bio_data()
    
    def estimate_emotion_rest_base(self):
        emo_name = 'Rest'
        
        if self.FLAG_THRESHOLD != 0:
            # Happy
            if self.beta_l_alpha_l >= self.THRESHOLD_AROUSAL and self.pnn50 >= self.THRESHOLD_VALENCE:
                emo_name = 'Happy'   
            # Relax
            elif self.beta_l_alpha_l < self.THRESHOLD_AROUSAL and self.pnn50 >= self.THRESHOLD_VALENCE:
                emo_name = 'Relax'
            # Sadness
            elif self.beta_l_alpha_l < self.THRESHOLD_AROUSAL and self.pnn50 < self.THRESHOLD_VALENCE:
                emo_name = 'Sadness'
            # Anger
            elif self.beta_l_alpha_l >= self.THRESHOLD_AROUSAL and self.pnn50 < self.THRESHOLD_VALENCE:
                emo_name = 'Anger'
        
        return [self.stimu, emo_name, self.beta_l_alpha_l, self.pnn10, self.pnn20, self.pnn30, self.pnn40, self.pnn50]
    
    def estimate_emotion_fix(self):
        # Happy
        if self.beta_l_alpha_l >= self.THRESHOLD_AROUSAL_fix and self.pnn50 >= self.THRESHOLD_VALENCE_fix:
            emo_name = 'Happy'   
        # Relax
        elif self.beta_l_alpha_l < self.THRESHOLD_AROUSAL_fix and self.pnn50 >= self.THRESHOLD_VALENCE_fix:
            emo_name = 'Relax'
        # Sad
        elif self.beta_l_alpha_l < self.THRESHOLD_AROUSAL_fix and self.pnn50 < self.THRESHOLD_VALENCE_fix:
            emo_name = 'Sad'
        # Angry
        elif self.beta_l_alpha_l >= self.THRESHOLD_AROUSAL_fix and self.pnn50 < self.THRESHOLD_VALENCE_fix:
            emo_name = 'Angry'
    
        return [self.stimu, emo_name, self.beta_l_alpha_l, self.pnn10, self.pnn20, self.pnn30, self.pnn40, self.pnn50]
    
    
    def publish_emo_status(self):
        if self.time_count.data < self.REST_TIME:
            self.stimu = 'Rest1'
            
        elif self.time_count.data < self.REST_TIME*2:
            self.stimu = 'Rest2'
            
            self.pnn10_rest.append(self.pnn10)
            self.pnn20_rest.append(self.pnn20)
            self.pnn30_rest.append(self.pnn30)
            self.pnn40_rest.append(self.pnn40)
            self.pnn50_rest.append(self.pnn50)
            self.rmssd_rest.append(self.rmssd)
            self.beta_l_alpha_l_rest.append(self.beta_l_alpha_l)             
            
        emo_and_bio = self.estimate_emotion_rest_base()
        
        self.emo_status.data = emo_and_bio[0]
        self.pub_emo_status.publish(self.emo_status)
            
        if self.time_count.data == self.REST_TIME*2:
            self.stimu = 'Stimu1'
            
            self.FLAG_THRESHOLD += 1
            self.THRESHOLD_VALENCE = sum(self.pnn50_rest) / self.REST_TIME
            self.THRESHOLD_AROUSAL = sum(self.beta_l_alpha_l_rest) / self.REST_TIME
            self.get_logger().info(f'THRESHOLD_VALENCE, {self.THRESHOLD_VALENCE}')
            self.get_logger().info(f'THRESHOLD_AROUSAL, {self.THRESHOLD_AROUSAL}')
            
            emo_and_bio = self.estimate_emotion_rest_base()
        
            self.emo_status.data = emo_and_bio[0]
            self.pub_emo_status.publish(self.emo_status)
        
        '''
        if self.REST_TIME*2 <= self.time_count.data < self.REST_TIME*3:
            self.stimu = 'Stimu1'
            
            self.FLAG_THRESHOLD += 1
            self.THRESHOLD_VALENCE = sum(self.pnn50_rest) / self.REST_TIME
            self.THRESHOLD_AROUSAL = sum(self.beta_l_alpha_l_rest) / self.REST_TIME
            self.get_logger().info(f'THRESHOLD_VALENCE, {self.THRESHOLD_VALENCE}')
            self.get_logger().info(f'THRESHOLD_AROUSAL, {self.THRESHOLD_AROUSAL}')
            
            emo_and_bio = self.estimate_emotion_rest_base()
        
            self.emo_status.data = emo_and_bio[0]
            self.pub_emo_status.publish(self.emo_status)
        
        if self.REST_TIME*3 <= self.time_count.data < self.REST_TIME*4:
            self.stimu = 'Robot1'
            
            emo_and_bio = self.estimate_emotion_rest_base()
        
            self.emo_status.data = emo_and_bio[1]
            self.pub_emo_status.publish(self.emo_status)
            print(emo_and_bio[1])
        
        if self.REST_TIME*4 <= self.time_count.data < self.REST_TIME*5:
            self.stimu = 'Rest2'
            
            emo_and_bio = self.estimate_emotion_rest_base()
        
            self.emo_status.data = emo_and_bio[0]
            self.pub_emo_status.publish(self.emo_status)
            
        if self.REST_TIME*5 <= self.time_count.data < self.REST_TIME*6:
            self.stimu = 'Stimu2'
            
            emo_and_bio = self.estimate_emotion_fix()
        
            self.emo_status.data = emo_and_bio[0]
            self.pub_emo_status.publish(self.emo_status)
        
        if self.REST_TIME*6 <= self.time_count.data < self.REST_TIME*7:
            self.stimu = 'Robot2'
            
            emo_and_bio = self.estimate_emotion_fix()
        
            self.emo_status.data = emo_and_bio[1]
            self.pub_emo_status.publish(self.emo_status)
        
        '''
        self.time_count.data += 1
        self.get_logger().info(f'time_count {self.time_count.data}')
        self.pub_time_count.publish(self.time_count)
        
        self.write_emo_data(emo_and_bio[0], emo_and_bio[1], emo_and_bio[2], emo_and_bio[3], emo_and_bio[4], emo_and_bio[5], emo_and_bio[6], emo_and_bio[7])  
        
        self.get_logger().info(f'Stimu: {emo_and_bio[0]}, Emotion: {emo_and_bio[1]}, lowb/a: {emo_and_bio[2]}, pnn50: {emo_and_bio[7]}')
        self.get_logger().info(f'THRESHOLD_VALENCE, {self.THRESHOLD_VALENCE}')
        self.get_logger().info(f'THRESHOLD_AROUSAL, {self.THRESHOLD_AROUSAL}\n\n')
    
    def write_emo_data(self, stimu, emo, b_a, pnn10, pnn20, pnn30, pnn40, pnn50):
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        self.emo_csv_writer.writerow([timestamp, stimu, emo, b_a, pnn10, pnn20, pnn30, pnn40, pnn50, self.THRESHOLD_AROUSAL, self.THRESHOLD_VALENCE])
        self.emo_csv_file.flush()
    
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
