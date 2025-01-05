import rclpy
from rclpy.node import Node
from my_custom_message.msg import BrainData
from std_msgs.msg import Float32
import numpy as np
import datetime
import time
from mne.filter import filter_data
import pandas as pd
from mne.time_frequency import psd_array_multitaper
from brainflow.board_shim import BoardShim, BrainFlowInputParams, BoardIds, BrainFlowPresets
from brainflow.data_filter import DataFilter

MAX_LEN_DATA = 30
X_PNN = 50

# 周波数帯域の設定
LOW_ALPHA_BAND = (8, 9)  # 低アルファ波
LOW_BETA_BAND = (13, 17)  # 低ベータ波

SAMPLING_RATE = 256  # サンプリングレート（Muse S の場合）
EEG_CHANNELS = [1, 2, 3, 4]  # EEGチャンネル
'''
- シータ、アルファ、ベータ、ガンマの周波数帯域の制限は、それぞれ 4 ～ 7.9 Hz、8 ～ 12.9 Hz、13 ～ 30 Hz、30.1 ～ 80 Hz
    - これらの制限は COBIDAS-MEEG [19](https://www.nature.com/articles/s41597-023-02525-0#ref-CR19)ガイドラインによって定義

'''

def read_mac_address():
    file_path = "/home/user/museS_address.txt"
    try:
        with open(file_path, "r") as f:
            address = f.readline().strip()
            return address
    except FileNotFoundError:
        print(f"Error: {file_path} が見つかりません。")
        return None

def compute_band_power(data, sfreq, band):
    """
    周波数帯域のパワーを計算する。
    """
    psds, freqs = psd_array_multitaper(data, sfreq=sfreq, fmin=band[0], fmax=band[1], verbose=False)
    return psds.mean(axis=-1)

class MuseSNode(Node):
    def __init__(self):
        super().__init__('museS_node')
        
        params = BrainFlowInputParams()
        params.mac_address = read_mac_address()

        self.board = BoardShim(BoardIds.MUSE_S_BOARD, params)
        self.board.prepare_session()

        self.board.config_board("p50")
        #board.config_board("p61")

        self.board.start_stream()
        time.sleep(10)
        
        self.pub_pnnx = self.create_publisher(Float32, 'pnnx', 10)
        self.pnnx = Float32() # pnnx
        self.pnnx.data = 0.0 # 送信データ
        # pnnxの計算に使用
        self.rri = 0
        self.rri_arr = []
        self.xx_count = 0

        # 安静時のpnnx
        self.pub_pnnx_rest_ave = self.create_publisher(Float32, 'pnnx_rest_ave', 10)
        self.pnnx_rest_ave = Float32() # 何も無い時(安静時)のpnnx平均
        self.pnnx_rest_ave.data = -1.0 # 送信データ
        # 安静時のpnnxの計算に使用
        self.pnnx_rest = [] # 何も無い時のpnnxデータ(30個)
    
    def publish_pnnx(self):
        sensor_data = self.sensor.readline().decode(encoding='utf-8').strip()
        if sensor_data.startswith('Q'):
            self.get_logger().info(f'Sensor_data:{sensor_data}')
            # self.get_logger().info(f'安静時平均：{self.pnnx_rest_ave.data}, 直前平均：{self.pnnx_stimuli_ave}')
            self.rri = float(sensor_data[1:])
            self.xx_count += 1
            
            # データ数が足りてる時はpnnx計算
            if self.xx_count >= MAX_LEN_DATA + 2:
                self.rri_arr.pop(0)
                self.rri_arr.append(self.rri)
                count = 0
                for i in range(MAX_LEN_DATA):
                    if abs(self.rri_arr[i] - self.rri_arr[i+1]) > X_PNN:
                        count += 1

                # pnnxを計算して更新、publish
                self.pnnx.data = float(count / 30)
                self.pub_pnnx.publish(self.pnnx)
                # 安静時平均をpublish
                self.publish_pnnx_rest_ave(self.pnnx)
                time_now = datetime.datetime.now()
                self.get_logger().info(f'Pnnx: {self.pnnx.data}, Time: {time_now}')

            # pnnxに必要なデータ数が足りない時は貯める
            elif self.xx_count < MAX_LEN_DATA + 2:
                self.rri_arr.append(self.rri)
                self.get_logger().info(f'RRI_data_len: {len(self.rri_arr)}')
        
    def publish_pnnx_rest_ave(self, pnnx):
        # 安静時データを貯める
        if len(self.pnnx_rest) < MAX_LEN_DATA:
            self.pnnx_rest.append(pnnx.data)  # pnnx.data をリストに追加
        # 安静時データが貯まっていてまだ平均を出していなければ出す
        elif self.pnnx_rest_ave.data == -1:
            self.pnnx_rest_ave.data = float(np.mean(self.pnnx_rest))
            self.pub_pnnx_rest_ave.publish(self.pnnx_rest_ave)
    
    def run(self):
        while rclpy.ok():
            self.publish_pnnx()

def main(args=None):
    rclpy.init(args=args)
    talker = MuseSNode()
    try:
        talker.run()
    except KeyboardInterrupt:
        pass
    finally:
        talker.board.stop_stream()
        talker.board.release_session()
        talker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
