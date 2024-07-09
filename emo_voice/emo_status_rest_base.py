import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, String
from my_custom_message.msg import PulseData, BrainData
import math
import csv
from datetime import datetime

class EmoStatusNode(Node):
    def __init__(self):
        super().__init__('emo_status_node')
        
        self.create_subscription(PulseData, 'pulse', self.pulse_callback, 10)
        self.create_subscription(BrainData, 'brain_wave', self.beta_l_alpha_l_callback, 10)
        
        self.pnn10 = 0.0 # valence
        self.pnn20 = 0.0
        self.pnn30 = 0.0
        self.pnn40 = 0.0
        self.pnn50 = 0.0
        self.rmssd = 0.0
        self.beta_l_alpha_l = 0 # arousal

        self.pub_emo_status = self.create_publisher(String, 'emo_status', 10)
        self.emo_status = String() # 感情状態
        self.emo_status.data = '' # 送信データ
        
        self.THRESHOLD_VALENCE = -1.0 # valenceの平均指標
        self.THRESHOLD_AROUSAL = -1.0 # arousalの平均指標
        self.FLAG_THRESHOLD = 0
        
        self.pnn10_rest = []
        self.pnn20_rest = []
        self.pnn30_rest = []
        self.pnn40_rest = []
        self.pnn50_rest = []
        self.rmssd_rest = []
        self.beta_l_alpha_l_rest = []
        
        self.rest_count = 0
        self.REST_TIME = 30

        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.publish_emo_status)
    
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_filename = f'/home/user/ros2_ws/src/emo_voice/bio_data/{timestamp}.csv'
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'beta_l_alpha_l', 'pnn50'])
        
        self.emo_csv_filename = f'/home/user/ros2_ws/src/emo_voice/emo__data/{timestamp}.csv'
        self.emo_csv_file = open(self.emo_csv_filename, mode='w', newline='')
        self.emo_csv_writer = csv.writer(self.emo_csv_file)
        self.emo_csv_writer.writerow(['timestamp', 'beta_l_alpha_l', 'pnn50', 'emo', 'threshold_b_a', 'threshold_pnn'])
    
    def write_bio_data(self):
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        self.csv_writer.writerow([timestamp, self.beta_l_alpha_l, self.pnn50])
        self.csv_file.flush()
        
    def pulse_callback(self, msg):
        self.pnn10 = msg.pnn10 # valence
        self.pnn20 = msg.pnn20
        self.pnn30 = msg.pnn30
        self.pnn40 = msg.pnn40
        self.pnn50 = msg.pnn50
        self.rmssd = msg.rmssd
        self.get_logger().info(f'PNN50: {self.pnn50}')
        
        self.write_bio_data()
    
    def beta_l_alpha_l_callback(self, msg):
        self.beta_l_alpha_l = msg.beta_l / msg.alpha_l
        self.get_logger().info(f'LOWBETA / LOWALPHA {self.beta_l_alpha_l}, {msg.beta_l}, {msg.alpha_l}')
        
        if self.rest_count < self.REST_TIME:
            self.rest_count += 1
            self.pnn10_rest.append(self.pnn10)
            self.pnn20_rest.append(self.pnn20)
            self.pnn30_rest.append(self.pnn30)
            self.pnn40_rest.append(self.pnn40)
            self.pnn50_rest.append(self.pnn50)
            self.rmssd_rest.append(self.rmssd)
            self.beta_l_alpha_l_rest.append(self.beta_l_alpha_l)
            self.get_logger().info(f'rest_count {self.rest_count}')
            
            if self.rest_count == self.REST_TIME and self.FLAG_THRESHOLD == 0:
                self.FLAG_THRESHOLD += 1
                self.THRESHOLD_VALENCE = sum(self.pnn50_rest) / self.REST_TIME
                self.THRESHOLD_AROUSAL = sum(self.beta_l_alpha_l_rest) / self.REST_TIME
                self.get_logger().info(f'THRESHOLD_VALENCE, {self.THRESHOLD_VALENCE}')
                self.get_logger().info(f'THRESHOLD_AROUSAL, {self.THRESHOLD_AROUSAL}')

        self.write_bio_data()
    
    def estimate_emotion(self):
        emo_name = ""
        
        # Happy
        if self.beta_l_alpha_l >= self.THRESHOLD_AROUSAL and self.pnn50 >= self.THRESHOLD_VALENCE:
            emo_name = "Happy"   
        # Relax
        elif self.beta_l_alpha_l < self.THRESHOLD_AROUSAL and self.pnn50 >= self.THRESHOLD_VALENCE:
            emo_name = "Relax"
        # Sad
        elif self.beta_l_alpha_l < self.THRESHOLD_AROUSAL and self.pnn50 < self.THRESHOLD_VALENCE:
            emo_name = "Sad"
        # Angry
        elif self.beta_l_alpha_l >= self.THRESHOLD_AROUSAL and self.pnn50 < self.THRESHOLD_VALENCE:
            emo_name = "Angry"
        
        return [emo_name, self.beta_l_alpha_l, self.pnn50]
    
    def publish_emo_status(self):
        if self.FLAG_THRESHOLD != 0:
            emo_and_bio = self.estimate_emotion()
            
            self.emo_status.data = emo_and_bio[0]
            self.pub_emo_status.publish(self.emo_status)
            
            self.write_emo_data(emo_and_bio[1], emo_and_bio[2], emo_and_bio[0])
            
            self.get_logger().info(f'Emotion: {str(sub_emo[0])}, Value: {str(sub_emo[1])}\n\n')
            self.get_logger().info(f'THRESHOLD_VALENCE, {self.THRESHOLD_VALENCE}')
            self.get_logger().info(f'THRESHOLD_AROUSAL, {self.THRESHOLD_AROUSAL}')
    
    def write_emo_data(self, b_a, pnn, emo):
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        self.emo_csv_writer.writerow([timestamp, b_a, pnn, emo, self.THRESHOLD_AROUSAL, self.THRESHOLD_VALENCE])
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
