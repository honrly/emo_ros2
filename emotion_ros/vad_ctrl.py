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

import torch
from scipy.io.wavfile import write
from collections import deque
import os

import threading
from collections import deque

from dataclasses import dataclass

GPU_SERVER_URL = 'http://192.168.65.146:5000/'

SAMPLE_RATE = 16000
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


def load_vad_model():
    model, utils = torch.hub.load(repo_or_dir='snakers4/silero-vad', model='silero_vad', force_reload=False)
    (get_speech_timestamps, _, _, _, _) = utils
    return model, get_speech_timestamps

def play_audio(audio_data):
    tmp = pygame.mixer.Sound(buffer=audio_data)
    pygame.mixer.Sound.play(tmp)
    # 再生が終わるまで待機
    while pygame.mixer.get_busy():
        pygame.time.Clock().tick(10)

def save_wave(file, data):
    with wave.open(file, 'wb') as wf:
        wf.setnchannels(CHANNEL)
        wf.setsampwidth(audio.get_sample_size(FORMAT))
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(b''.join(data))

class TalkCtrl(Node):
    def __init__(self):
        super().__init__('vad_ctrl')
        
        self.create_subscription(String, 'recog_emo', self.emo_callback, 10)
        self.emo = ''
        self.flag_emosub = False
        
        # voicevoxの場合24000指定なので、音声再生の初期化
        # pygame.mixer.pre_init(frequency=24000, size=-16, channels=1)
        pygame.init()
        
        open_jtalk=['open_jtalk']
        mech=['-x','/var/lib/mecab/dic/open-jtalk/naist-jdic']
        htsvoice=['-m','/usr/share/hts-voice/mei/mei_normal.htsvoice']
        speed=['-r','1.0']
        self.cmd = open_jtalk + mech + htsvoice + speed
        
        self.output_dir = f"{os.getcwd()}/src/emotion_ros/output_audio/"
        os.makedirs(self.output_dir, exist_ok=True)

    def emo_callback(self, msg):
        self.emo = msg.data
        self.flag_emosub = True
        self.get_logger().info(f"Recog_emotion: {self.emo}")
      
    def play_jtalk(self, audio_data, total_time):
        start_time = time.time()
        pygame.mixer.quit()  # ミキサーを初期化し直す

        # バイト列をWaveオブジェクトとして読み込む
        with io.BytesIO(audio_data) as audio_io:
            with wave.open(audio_io, 'rb') as wf:
                n_channels = wf.getnchannels()
                sample_width = wf.getsampwidth()
                frame_rate = wf.getframerate()
                audio_frames = wf.readframes(wf.getnframes())

        # numpyでデータを変換
        if sample_width == 2:
            dtype = np.int16
        else:
            raise ValueError("Unsupported sample width: %d" % sample_width)
        audio_array = np.frombuffer(audio_frames, dtype=dtype)

        # pygame.mixerを適切なフォーマットで初期化
        pygame.mixer.init(frequency=frame_rate, size=-8 * sample_width, channels=n_channels)

        # バイト列に戻してpygameで再生
        tmp = pygame.mixer.Sound(audio_array.tobytes())
        end_time = time.time()
        self.get_logger().info("再生時間: {:.2f} seconds".format(end_time - start_time))
        
        total_time += end_time - start_time
        self.get_logger().info("合計時間: {:.2f} seconds".format(total_time))
        
        tmp.play()

        # 再生が終わるまで待機
        while pygame.mixer.get_busy():
            pygame.time.Clock().tick(10)
      
      
    def jtalk(self, tt):
        start_time = time.time()

        # 一時ファイルを作成
        with tempfile.NamedTemporaryFile(delete=False, suffix='.wav') as temp_wav:
            temp_wav_path = temp_wav.name

        # open_jtalk コマンドを準備してファイルに音声を保存
        cmd = self.cmd + ['-ow', temp_wav_path]
        subprocess.run(cmd, input=tt.encode())
        end_time = time.time()
        self.get_logger().info("音声合成: {:.2f} seconds, {}文字".format(end_time - start_time, len(tt))) 

        # 音声ファイルを再生
        subprocess.run(['aplay', '-q', temp_wav_path])
    
    def send_audio(self, file_name, output_path):
        file = {'file': (f"{file_name}", open(f"{output_path}", 'rb'), 'audio/wav')}
        emo = {'emo': f'{self.emo}'}
        self.get_logger().info(f"Send_emotion: {self.emo}")
        
        
        try:
            start_time = time.time()
            # データを送信
            response = requests.post(GPU_SERVER_URL, files=file, data=emo)
            file['file'][1].close()
            if response.status_code == 200:
                end_time = time.time()
                voice = base64.b64decode(response.json()['voice'])
                
                self.get_logger().info(f"Your Voice: {response.json()['input']}")
                self.get_logger().info(f"Received from Flask: {response.json()['text']}")
                self.get_logger().info("生成時間: {:.2f} seconds, {}文字".format(end_time - start_time, len(response.json()['text'])))
                total_time = end_time - start_time
                
                self.play_jtalk(voice, total_time)
            else:
                self.get_logger().info(f"Error: {response.status_code}")
        except Exception as e:
            self.get_logger().info(f"Request failed: {e}")
    
    def voice_ad(self):
        global is_speaking, silence_start_time
        
        model, get_speech_timestamps = load_vad_model()
        while True:
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

    def audio_record(self):
        global recorded_data
        file_count = 0
        
        stream = audio.open(format=FORMAT, channels=CHANNEL, rate=SAMPLE_RATE, 
                            input=True, input_device_index=3,frames_per_buffer=CHUNK_SIZE)
        self.get_logger().info("Thread A: マイク起動")
        try:
            if self.flag_emosub:
                self.jtalk("こんにちは")
            while rclpy.ok():
                data = stream.read(CHUNK_SIZE, exception_on_overflow=False)
                with lock:
                    audio_array = np.frombuffer(data, dtype=np.int16)
                    if is_speaking:
                        recorded_data.append(data)
                    buffer.append(audio_array)

                # 発話終了のシグナルを受信
                if event.is_set():
                    with lock:
                        file_count += 1
                        file_name = f"speech_{file_count}.wav"
                        file_path = self.output_dir + file_name
                        save_wave(file_path, recorded_data)
                        self.get_logger().info(f"Thread A: 保存{file_name}")
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
