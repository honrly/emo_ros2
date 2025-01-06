import rclpy
from rclpy.node import Node
from my_custom_message.msg import PulseData
from std_msgs.msg import Float32
import numpy as np
import math
import datetime
import time
import pandas as pd

# 定数
REST = 180
IBI_SIZE = 31

def read_mac_address():
    file_path = "/home/user/museS_address.txt"
    try:
        with open(file_path, "r") as f:
            address = f.readline().strip()
            return address
    except FileNotFoundError:
        print(f"Error: {file_path} が見つかりません。")
        return None


class ReceiveRRINode(Node):
    def __init__(self):
        super().__init__('receive_rri_node')
        
        self.pulse = PulseData()
        self.pulse.bpm = 0
        
        self.pub_pnnx = self.create_publisher(PulseData, 'pulse', 10)
        
        self.ibi_list = [0] * IBI_SIZE
        self.ibi_count = 0
        self.RRI_data = []

        # 安静時のpnnx
        self.pub_pnnx_rest_ave = self.create_publisher(Float32, 'pnnx_rest_ave', 10)
        self.pnnx_rest_ave = Float32() # 何も無い時(安静時)のpnnx平均
        self.pnnx_rest_ave.data = -1.0 # 送信データ
        # 安静時のpnnxの計算に使用
        self.pnnx_rest = []
    
    def publish_pnnx(self):
        for ibi in self.RRI_data:
            self.pulse.ibi = ibi  # 最後の値をメッセージに記録

            if self.ibi_count >= IBI_SIZE:
                self.ibi_list.pop(0)  # 最も古い値を削除
                self.ibi_list.append(ibi)  # 新しい値を追加
            elif self.ibi_count >= 0:
                self.ibi_list[self.ibi_count] = ibi  # 配列に値を追加
                self.ibi_count += 1
            else:
                raise Exception("IBI count error")

        
        self.calc_hrv()
        self.pub_pnnx.publish(self.pnnx)
        # 安静時平均をpublish
        self.publish_pnnx_rest_ave()
        time_now = datetime.datetime.now()
        self.get_logger().info(f'Pnnx: {self.pnnx.data}, Time: {time_now}')

        
    def publish_pnnx_rest_ave(self):
        # 安静時データを貯める
        if len(self.pnnx_rest) < REST:
            self.pnnx_rest.append(self.pulse.pnn20)  # pnnx.data をリストに追加
        # 安静時データが貯まっていてまだ平均を出していなければ出す
        elif self.pnnx_rest_ave.data == -1.0:
            self.pnnx_rest_ave.data = float(np.mean(self.pnnx_rest))
            self.pub_pnnx_rest_ave.publish(self.pnnx_rest_ave)
    
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
        global msg
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
