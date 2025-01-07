import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

import requests
import time
import pyaudio
import pygame
import base64
import numpy as np
import sounddevice # alsa関連のwarnログが出るのを回避
import io
import wave
import subprocess
import tempfile
import json

import torch
from scipy.io.wavfile import write
from collections import deque
import os

import random
from datetime import datetime
from zoneinfo import ZoneInfo

GPU_SERVER_URL = 'https://square-macaque-thankfully.ngrok-free.app/recog_only'

SAMPLE_RATE = 16000
#SAMPLE_RATE = 48000
CHANNEL = 1  # チャンネル1(モノラル), ReSpeaker 4 Mic Array = max 6だけど音声検知をリアルタイムで処理するにはラズパイだと馬鹿多いので1で良い
CHUNK_SIZE = 2048 # ラズパイは2048、1回のデータ取得量
BUFFER_SIZE = 10 # どれくらい貯めるか
SILENCE_DURATION = 1.0  # 発話終了と判定する無音時間

audio = pyaudio.PyAudio()
FORMAT = pyaudio.paInt16

lock = threading.Lock()
event = threading.Event()

buffer = deque(maxlen=BUFFER_SIZE)
recorded_data = []
is_speaking = False
silence_start_time = None

home_dir = os.environ["HOME"]
os.system("setterm -cursor off>/dev/tty0")

def disp(img_file):
    command1 = "cat " + home_dir + "/turtlebot3_ws/src/emotion_ros/face_DB/" + img_file + ".raw > /dev/fb0"
    command2 = "cat /dev/fb0 > /home/ubuntu/tmp.raw"
    os.system(command1)



def load_vad_model():
    model, utils = torch.hub.load(repo_or_dir='snakers4/silero-vad', model='silero_vad', force_reload=False)
    (get_speech_timestamps, _, _, _, _) = utils
    return model, get_speech_timestamps


def play_wave(directory, wav_file):
    file_path = os.path.join(directory, wav_file)
    sound = pygame.mixer.Sound(file_path)
    sound.play()
    # 再生が終わるまで待機
    time.sleep(sound.get_length())
    # 再生の終了
    sound.stop()

def save_wave(file, data):
    with wave.open(file, 'wb') as wf:
        wf.setnchannels(CHANNEL)
        wf.setsampwidth(audio.get_sample_size(FORMAT))
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(b''.join(data))

class TalkCtrl(Node):
    def __init__(self):
        super().__init__('vad_ctrl')
        
        self.shutdown_flag = threading.Event()
        self.create_subscription(String, 'recog_emo', self.emo_callback, 10)
        self.emo = ''
        
        self.create_subscription(String, 'emo_status', self.exp_manage_callback, 10)
        
        # voicevoxの場合24000指定なので、音声再生の初期化
        # pygame.mixer.pre_init(frequency=24000, size=-16, channels=1)
        pygame.init()
        
        
        self.output_dir = "/home/user/turtlebot3_ws/src/emotion_ros/output_audio/"
        os.makedirs(self.output_dir, exist_ok=True)
        
        
        self.voice_dir = "/home/user/turtlebot3_ws/src/emotion_ros/Voice_JP/"
        self.hello_wave = "こんにちは_normal.wav"
        self.bye_wave = "あ、時間が来たので今回はこれでおしまいです。またね_normal.wav"
        
        
        # Record txt, csvだと文字数が怪しいので
        directory_path = '/home/user/turtlebot3_ws/src/emotion_ros'
        speech_data_path = os.path.join(directory_path, 'data_record/speech_data')
        os.makedirs(speech_data_path, exist_ok=True)

        timestamp = datetime.now(ZoneInfo("Asia/Tokyo")).strftime('%Y%m%d_%H%M%S')

        self.txt_filename = os.path.join(speech_data_path, f'{timestamp}_speech.txt')
        self.txt_file = open(self.txt_filename, mode='w', newline='')
        self.txt_file.write('user_time,user_text,user_len,recog_emo\n')
        
        self.user_time = None
        self.user_text = ""
        self.user_len = 0
        self.csv_emo = ""
        
        
    def write_speech_data(self):
        self.txt_file.write(f'{self.user_time},{self.user_text},{self.user_len},{self.csv_emo}\n')
        self.txt_file.flush()
    
    def exp_manage_callback(self, msg):
        if msg.data == 'Rest1':
            disp("initial")
        elif msg.data == 'Robot':
            disp("happy1")
        elif msg.data == 'Rest2':
            self.shutdown_flag.set()
            self.get_logger().info("終了")
            
            while pygame.mixer.get_busy():
                time.sleep(0.1)
            play_wave(self.voice_dir, self.bye_wave)
            disp("initial")
            while True:
                time.sleep(1)
            
    
    def emo_callback(self, msg):
        self.emo = msg.data
        self.get_logger().info(f"Recog_emotion: {self.emo}")
      

    def send_audio(self, file_name, output_path):
        file = {'file': (f"{file_name}", open(f"{output_path}", 'rb'), 'audio/wav')}
        emo = {'emo': f'{self.emo}'}
        self.get_logger().info(f"Send_emotion: {emo['emo']}")
        self.csv_emo = emo['emo']

        try:
            response = requests.post(GPU_SERVER_URL, files=file, data=emo)
            file['file'][1].close()
            
            with wave.open(f"{output_path}", mode='rb') as wf:
                self.user_len = int(wf.getnframes() / wf.getframerate())

            if response.status_code == 200:
                self.get_logger().info(f"認識した音声: {response.json()['input']}")
                self.user_text = response.json()['input']
            else:
                self.get_logger().info(f"Error: {response.status_code}")
        except Exception as e:
            self.get_logger().info(f"Request failed: {e}")
        finally:
            print('finally')
            self.write_speech_data()
            
    # VADの判定
    def voice_ad(self):
        global is_speaking, silence_start_time
        
        model, get_speech_timestamps = load_vad_model()
        while not self.shutdown_flag.is_set():
            time.sleep(0.1)  # チェック間隔
            with lock:
                if len(buffer) > 0:
                    # 音声データを処理
                    audio_tensor = torch.from_numpy(np.concatenate(buffer)).float() / 32768.0  # Normalize audio data

                    # 発話のタイムスタンプ
                    speech_timestamps = get_speech_timestamps(audio_tensor, model, sampling_rate=SAMPLE_RATE, threshold=0.5)

                    if speech_timestamps:
                        # 発話開始
                        if not is_speaking:
                            self.user_time = datetime.now(ZoneInfo("Asia/Tokyo")).strftime('%H:%M:%S.%f')[:-3]
                            self.get_logger().info("Thread B: 発話検知、録音開始")
                            start_time = time.time()
                            for buf_data in buffer:
                                recorded_data.append(buf_data.tobytes())
                        is_speaking = True # 発話中
                        silence_start_time = None  # 無音時間リセット
                    else:
                        # 無音が一定時間継続した場合に発話終了と判定
                        if is_speaking:
                            if silence_start_time is None:
                                silence_start_time = time.time()
                            elif (time.time() - silence_start_time >= SILENCE_DURATION) or (time.time() - start_time >= 29):
                                self.get_logger().info("Thread B: 発話終了")
                                is_speaking = False
                                buffer.clear()
                                event.set()  # 発話終了のシグナルを送信
                        else:
                            silence_start_time = None  # 無音時間リセット

    # メインのループ
    def audio_record(self):
        global recorded_data
        file_count = None
        
        stream = audio.open(format=FORMAT, channels=CHANNEL, rate=SAMPLE_RATE, 
                            input=True, input_device_index=3,frames_per_buffer=CHUNK_SIZE)
        self.get_logger().info("Thread A: マイク起動")
        try:
            if True:
                play_wave(self.voice_dir, self.hello_wave)
                while not self.shutdown_flag.is_set():
                    data = stream.read(CHUNK_SIZE, exception_on_overflow=False)
                    with lock:
                        audio_array = np.frombuffer(data, dtype=np.int16)
                        if is_speaking:
                            recorded_data.append(data)
                        buffer.append(audio_array)

                    # 発話終了のシグナルを受信
                    if event.is_set():
                        with lock:
                            file_count = datetime.now(ZoneInfo("Asia/Tokyo")).strftime('%Y%m%d_%H%M%S')
                            file_name = f"speech_{file_count}.wav"
                            file_path = self.output_dir + file_name
                            
                            save_wave(file_path, recorded_data)
                            self.get_logger().info(f"Thread A: 保存{file_name}")
                            
                            # 録音した音声をwavファイルで送信
                            self.send_audio(file_name, file_path)
                            recorded_data.clear()
                        event.clear()
                        buffer.clear()
        finally:
            stream.stop_stream()
            stream.close()
    
    def start(self):        
        thread_a = threading.Thread(target=self.audio_record, daemon=True)
        thread_b = threading.Thread(target=self.voice_ad, daemon=True)
        
        thread_a.start()
        thread_b.start()

def main(args=None):
    try:
      rclpy.init(args=args)
      talker = TalkCtrl()
      talker.start()
      rclpy.spin(talker)
    except KeyboardInterrupt:
      pass
    finally:
      talker.destroy_node()
      rclpy.shutdown()
  
if __name__ == '__main__':
    main()
