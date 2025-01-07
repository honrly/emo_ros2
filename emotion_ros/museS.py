import rclpy
from rclpy.node import Node
from my_custom_message.msg import BrainData
from std_msgs.msg import Float32
import numpy as np
import time
from mne.filter import filter_data
import pandas as pd
import os
import csv
from datetime import datetime
from zoneinfo import ZoneInfo
from mne.time_frequency import psd_array_multitaper
from brainflow.board_shim import BoardShim, BrainFlowInputParams, BoardIds, BrainFlowPresets
from brainflow.data_filter import DataFilter


LOW_ALPHA_BAND = (8, 9)
LOW_BETA_BAND = (13, 17)

SAMPLING_RATE = 256
EEG_CHANNELS = [1, 2, 3, 4]
'''
- シータ、アルファ、ベータ、ガンマの周波数帯域の制限は、それぞれ 4 ～ 7.9 Hz、8 ～ 12.9 Hz、13 ～ 30 Hz、30.1 ～ 80 Hz
    - これらの制限は COBIDAS-MEEG [19](https://www.nature.com/articles/s41597-023-02525-0#ref-CR19)ガイドラインによって定義

'''

RECONNECT_TIMEOUT = 2

def calc_band_power(data, sfreq, band):
    # 周波数帯のパワーを計算
    psds, freqs = psd_array_multitaper(data, sfreq=sfreq, fmin=band[0], fmax=band[1], verbose=False)
    return psds.mean(axis=-1)

class MuseSNode(Node):
    def __init__(self):
        super().__init__('museS_node')
        self.timestamp = None
        
        self.board = self.setup_muse()
        self.data_raw = None
        
        self.af7_data = []
        self.af8_data = []
        
        # Publish EEG
        self.pub_brain_wave = self.create_publisher(BrainData, 'brain_wave', 10)
        self.brain_wave = BrainData()
        self.brain_wave.poorsignal = 0
        self.brain_wave.delta = 0.0
        self.brain_wave.theta = 0.0
        self.brain_wave.alpha_l = 0.0
        self.brain_wave.alpha_h= 0.0
        self.brain_wave.beta_l = 0.0
        self.brain_wave.beta_h = 0.0
        self.brain_wave.gamma_l = 0.0
        self.brain_wave.gamma_m = 0.0

        directory_path = '/home/user'
        bio_eeg_data_path = os.path.join(directory_path, 'data_raw/eeg')
        os.makedirs(bio_eeg_data_path, exist_ok=True)

        self.csv_filename = os.path.join(bio_eeg_data_path, f'{self.timestamp}_eeg_raw.csv')
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'package_num_channel', 'TP9', 'AF7', 'AF8', 'TP10', 'other_channels', 'unix_timestamp', 'marker_channel'])
        
        self.last_data_time = time.time()
    
    def write_raw_data(self, raw):
        timestamp = datetime.now(ZoneInfo("Asia/Tokyo")).strftime('%Y-%m-%d %H:%M:%S.%f')[:-5]
        self.csv_writer.writerow([timestamp, raw])
        self.csv_file.flush()
     
    
    def setup_muse(self):
        params = BrainFlowInputParams()
        params.mac_address = self.read_mac_address()

        board = BoardShim(BoardIds.MUSE_S_BOARD, params)
        max_retries = 10
        for attempt in range(max_retries):
            try:
                board.prepare_session()
                self.get_logger().info("Board session prepared successfully.")
                break
            except Exception as e:
                self.get_logger().info(f"Attempt {attempt + 1}/{max_retries} failed: {e}")
                if attempt < max_retries - 1:
                    self.get_logger().info(f"Retrying in 2 seconds...")
                    time.sleep(2)
                else:
                    self.get_logger().info("Max retries reached. Failed to prepare board session.")

        board.config_board("p50")
        board_id = BoardIds.MUSE_S_BOARD.value
        self.get_logger().info(str(BoardShim.get_board_descr(board_id, 0)))

        board.start_stream()
        self.timestamp = datetime.now(ZoneInfo("Asia/Tokyo")).strftime('%Y%m%d_%H%M%S')
        time.sleep(10)
        return board
    
    def read_mac_address(self):
        file_path = "/home/user/museS_address.txt"
        try:
            with open(file_path, "r") as f:
                address = f.readline().strip()
                return address
        except FileNotFoundError:
            self.get_logger().info(f"Error: {file_path} が見つかりません。")
            return None
    
    def get_eegdata(self):
        self.data_raw = self.board.get_board_data(preset=BrainFlowPresets.DEFAULT_PRESET)
        self.write_raw_data(self.data_raw)
        if self.data_raw.size > 0:
            self.last_data_time = time.time()
            
            self.af7_data.extend(self.data_raw[EEG_CHANNELS[0]])
            self.af8_data.extend(self.data_raw[EEG_CHANNELS[1]])

            if len(self.af7_data) >= SAMPLING_RATE:
                # NumPy 配列に変換
                af7_array = np.array(self.af7_data)
                af8_array = np.array(self.af8_data)

                # 周波数解析
                low_alpha_power_af7 = calc_band_power(af7_array, SAMPLING_RATE, LOW_ALPHA_BAND)
                low_beta_power_af7 = calc_band_power(af7_array, SAMPLING_RATE, LOW_BETA_BAND)
                low_alpha_power_af8 = calc_band_power(af8_array, SAMPLING_RATE, LOW_ALPHA_BAND)
                low_beta_power_af8 = calc_band_power(af8_array, SAMPLING_RATE, LOW_BETA_BAND)

                self.get_logger().info(f"Low Alpha Power AF7: {low_alpha_power_af7}")
                self.get_logger().info(f"Low Beta Power AF7: {low_beta_power_af7}")
                self.get_logger().info(f"Low Alpha Power AF8: {low_alpha_power_af8}")
                self.get_logger().info(f"Low Beta Power AF8: {low_beta_power_af8}")
                
                self.brain_wave.alpha_l = (low_alpha_power_af7 + low_alpha_power_af8) / 2
                self.brain_wave.beta_l = (low_beta_power_af7 + low_beta_power_af8) / 2
                
                self.get_logger().info(f"Low Beta Power: {self.brain_wave.beta_l}")
                self.get_logger().info(f"Low Alpha Power: {self.brain_wave.alpha_l}")
                
                self.pub_brain_wave.publish(self.brain_wave)
                
                self.af7_data = []
                self.af8_data = []
        else:
            if time.time() - self.last_data_time > RECONNECT_TIMEOUT:
                self.get_logger().info("No data received for 2 seconds. Reconnect.")
                self.board.release_session()
                DataFilter.write_file(self.data_raw, f'{self.timestamp}_brainflow.csv', 'w')
                print(f'Save {self.timestamp}_brainflow.csv')
                self.board = self.setup_muse()
    
    def run(self):
        while rclpy.ok():
            self.get_eegdata()

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
        DataFilter.write_file(talker.data_raw, f'{talker.timestamp}_brainflow.csv', 'w')
        print(f'Save {talker.timestamp}_brainflow.csv')
        talker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
