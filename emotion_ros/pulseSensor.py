import rclpy
from rclpy.node import Node
from my_custom_message.msg import PulseData
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import serial
import numpy as np
import math
import os
import csv
from datetime import datetime
from zoneinfo import ZoneInfo


IBI_SIZE = 31

class PnnxNode(Node):
    def __init__(self):
        super().__init__('pnnx_node')
        
        self.sensor = serial.Serial('/dev/ttyACM0', 115200)
        
        self.pulse = PulseData()
        self.pulse.bpm = 0
        
        self.pub_pulse = self.create_publisher(PulseData, 'pulse', 10)
        
        self.ibi_list = [0] * IBI_SIZE
        self.ibi_count = 0
        
        directory_path = '/home/user'
        bio_pulse_data_path = os.path.join(directory_path, 'data_raw/pulse')
        os.makedirs(bio_pulse_data_path, exist_ok=True)

        timestamp = datetime.now(ZoneInfo("Asia/Tokyo")).strftime('%Y%m%d_%H%M%S')
        self.csv_filename = os.path.join(bio_pulse_data_path, f'{timestamp}_pulse_raw.csv')
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'bpm', 'ibi', 'sdnn', 'cvnn', 'rmssd', 'pnn10', 'pnn20', 'pnn30', 'pnn40', 'pnn50'])
   
    def write_raw_data(self):
        # 各行にタイムスタンプと生データを記録
        timestamp = datetime.now(ZoneInfo("Asia/Tokyo")).strftime('%H:%M:%S.%f')[:-3]
        self.csv_writer.writerow([timestamp, self.pulse.bpm, self.pulse.ibi, self.pulse.sdnn, self.pulse.cvnn, self.pulse.rmssd, 
                                        self.pulse.pnn10, self.pulse.pnn20, self.pulse.pnn30, self.pulse.pnn40, self.pulse.pnn50])
        self.csv_file.flush()
     
    
    def publish_pnnx(self):
        sensor_data = self.sensor.readline().decode(encoding='utf-8').strip()
        if sensor_data.startswith('Q'):
            self.get_logger().info(f'Sensor_data:{sensor_data}')
            # self.get_logger().info(f'安静時平均：{self.pnnx_rest_ave.data}, 直前平均：{self.pnnx_stimuli_ave}')
            self.pulse.ibi = int(sensor_data[1:])

            if self.ibi_count >= IBI_SIZE:
                self.ibi_list.pop(0)
                self.ibi_list.append(self.pulse.ibi)
            elif self.ibi_count >= 0:
                self.ibi_list[self.ibi_count] = self.pulse.ibi
                self.ibi_count += 1
            else:
                raise Exception("IBI count error")
            
            self.calc_hrv()
            self.write_raw_data()
            self.pub_pulse.publish(self.pulse)

    def calc_hrv(self):
        mean = sum(self.ibi_list) / IBI_SIZE
        self.calc_sdnn(mean)
        self.calc_rmssd()
        self.calc_pnn()

    def calc_sdnn(self, mean):
        variance_sum = sum((ibi - mean) ** 2 for ibi in self.ibi_list)
        sdnn = math.sqrt(variance_sum / IBI_SIZE)
        self.pulse.sdnn = sdnn

        self.calc_cvnn(mean, sdnn)

    def calc_cvnn(self, mean, sdnn):
        cvnn = sdnn / mean
        self.pulse.cvnn = cvnn

    def calc_rmssd(self):
        diff_squares_sum = sum((self.ibi_list[i] - self.ibi_list[i - 1]) ** 2 for i in range(1, IBI_SIZE))
        rmssd = math.sqrt(diff_squares_sum / IBI_SIZE)
        self.pulse.rmssd = rmssd

    def calc_pnn(self):
        nn10 = nn20 = nn30 = nn40 = nn50 = 0

        for i in range(1, IBI_SIZE):
            nnx = abs(self.ibi_list[i] - self.ibi_list[i - 1])
            if nnx > 50:
                nn50 += 1
            if nnx > 40:
                nn40 += 1
            if nnx > 30:
                nn30 += 1
            if nnx > 20:
                nn20 += 1
            if nnx > 10:
                nn10 += 1

        self.pulse.pnn50 = nn50 / (IBI_SIZE - 1)
        self.pulse.pnn40 = nn40 / (IBI_SIZE - 1)
        self.pulse.pnn30 = nn30 / (IBI_SIZE - 1)
        self.pulse.pnn20 = nn20 / (IBI_SIZE - 1)
        self.pulse.pnn10 = nn10 / (IBI_SIZE - 1)
    
    def run(self):
        while rclpy.ok():
            self.publish_pnnx()

def main(args=None):
    rclpy.init(args=args)
    talker = PnnxNode()
    try:
        talker.run()
    except KeyboardInterrupt:
        pass
    finally:
        talker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
