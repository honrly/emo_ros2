import rclpy
from rclpy.node import Node
from my_custom_message.msg import PulseData
from std_msgs.msg import Float32
import numpy as np
import math
import datetime
import time
import pandas as pd
from concurrent.futures import ThreadPoolExecutor
from bluepy.btle import Peripheral
import bluepy.btle as btle
import binascii
import os
import csv
from datetime import datetime
from zoneinfo import ZoneInfo

# 定数
REST = 180
IBI_SIZE = 31
RRI_data = []

class MyDelegate(btle.DefaultDelegate):
    def __init__(self, params):
        btle.DefaultDelegate.__init__(self)
 
    def handleNotification(self, cHandle, data):
        global RRI_data
        c_data = binascii.b2a_hex(data)
        ##「c_data」が受信したデータ，デフォルトは16進数標記なので、10進数に戻す必要がある
        hr = int(c_data[2:4],16)
        v1 = int(c_data[4:6],16)
        v2 = int(c_data[6:8],16)
        rri1 = (v2<<8) + v1
        RRI_data = np.hstack([RRI_data, rri1])
        print(rri1)
        if len(c_data) > 8:
            v3 = int(c_data[8:10],16)
            v4 = int(c_data[10:12],16)
            rri2 = (v4<<8) + v3
            RRI_data = np.hstack([RRI_data, rri2])
            print(rri2)
        print(RRI_data)
 
# センサークラス
class SensorBLE(Peripheral):
    def __init__(self, addr):
        Peripheral.__init__(self, addr, addrType="random")


class ReceiveRRINode(Node):
    def __init__(self):
        super().__init__('receive_rri_node')
        
        self.polar_mac_address = self.read_mac_address()
        
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
     
    
    def read_mac_address(self):
        file_path = "/home/user/polarH10_address.txt"
        try:
            with open(file_path, "r") as f:
                address = f.readline().strip()
                return address
        except FileNotFoundError:
            self.get_logger().info(f"Error: {file_path} が見つかりません。")
            return None
    
    def publish_pulse(self):
        global RRI_data
        while True:  # 再接続処理を繰り返すため
            try:
                medal = SensorBLE(self.polar_mac_address)
                medal.setDelegate(MyDelegate(btle.DefaultDelegate))
                
                # ノーティフィケーションを有効にする
                medal.writeCharacteristic(0x0011, b"\x01\x00", True)
                
                # データを受信し続ける
                while True:
                    if medal.waitForNotifications(1.0):  # 1秒間隔でデータを受け取る
                        if len(RRI_data) != 0:
                            for ibi in RRI_data:
                                self.pulse.ibi = int(ibi)

                                if self.ibi_count >= IBI_SIZE:
                                    self.ibi_list.pop(0)
                                    self.ibi_list.append(ibi)
                                elif self.ibi_count >= 0:
                                    self.ibi_list[self.ibi_count] = ibi
                                    self.ibi_count += 1
                                else:
                                    raise Exception("IBI count error")
                                
                                self.calc_hrv()
                                self.write_raw_data()
                                self.pub_pulse.publish(self.pulse)
                                
                            RRI_data = []
                        continue  # 一定時間で接続が切れないように継続
                    
            except btle.BTLEDisconnectError as e:
                self.get_logger().info(f"再接続: {e}")
                time.sleep(1)
                continue  # 再接続

            except ValueError as e:
                self.get_logger().info(f"データエラー: {e}")
                try:
                    medal.writeCharacteristic(0x0011, b"\x00\x00", True)
                    medal.disconnect()
                except Exception as e:
                    medal.writeCharacteristic(0x0011, b"\x00\x00", True)
                    medal.disconnect()
                time.sleep(1)

            except KeyboardInterrupt:
                self.get_logger().info("終了")
                break

            finally:
                try:
                    if 'medal' in locals() or 'medal' in globals():
                        medal.writeCharacteristic(0x0011, b"\x00\x00", True)
                        medal.disconnect()
                except Exception:
                    pass
    
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
            self.publish_pulse()

def main(args=None):
    rclpy.init(args=args)
    talker = ReceiveRRINode()
    try:
        talker.run()
    except KeyboardInterrupt:
        pass
    finally:
        talker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
