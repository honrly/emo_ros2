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
import pyaudio
from scipy.io.wavfile import write
from collections import deque
import os
import pickle

GPU_SERVER_URL = 'http://192.168.65.146:5000/'

def load_vad_model():
    model, utils = torch.hub.load(repo_or_dir='snakers4/silero-vad', model='silero_vad', force_reload=True)
    (get_speech_timestamps, _, _, _, _) = utils
    return model, get_speech_timestamps

def play_audio(audio_data):
    tmp = pygame.mixer.Sound(buffer=audio_data)
    pygame.mixer.Sound.play(tmp)
    # 再生が終わるまで待機
    while pygame.mixer.get_busy():
        pygame.time.Clock().tick(10)

class TalkCtrl(Node):
    def __init__(self):
        super().__init__('vad_ctrl')
        
        self.create_subscription(String, 'recog_emo', self.emo_callback, 10)
        self.emo = ''
        self.flag_emosub = False
        
        # 音声再生の初期化、voicevox
        #pygame.mixer.pre_init(frequency=24000, size=-16, channels=1)
        pygame.init()
        
        open_jtalk=['open_jtalk']
        mech=['-x','/var/lib/mecab/dic/open-jtalk/naist-jdic']
        htsvoice=['-m','/usr/share/hts-voice/mei/mei_normal.htsvoice']
        speed=['-r','1.0']
        self.cmd = open_jtalk + mech + htsvoice + speed

    def emo_callback(self, msg):
        self.emo = msg.data
        self.flag_emosub = True
        self.get_logger().info(f"Recog_emotion: {self.emo}")
    
    def talk(self):
        output_dir = f"{os.getcwd()}/src/emotion_ros/output_audio"
        os.makedirs(output_dir, exist_ok=True)
        model, get_speech_timestamps = load_vad_model()
        threading.Thread(target=self.send_msg_server(output_dir, model, get_speech_timestamps), daemon=True).start()
      
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
    
    def send_msg_server(self, output_dir, model, get_speech_timestamps):
        RATE = 16000
        CHUNK = 2048  # ラズパイは2048
        FORMAT = pyaudio.paInt16
        CHANNELS = 1  # チャンネル1(モノラル), ReSpeaker 4 Mic Array = max 6だけど音声検知をリアルタイムで処理するにはラズパイだと馬鹿多いので1で良い

        # マイク
        audio = pyaudio.PyAudio()
        stream = audio.open(format=FORMAT, channels=CHANNELS,
                            rate=RATE, input=True,
                            input_device_index=3,
                            frames_per_buffer=CHUNK)
        buffer = deque(maxlen=10)
        first_speech_audio = deque(maxlen=10) # 検知したあと音声データを保存すると最初が抜けおるちのでそれ用
        speech_audio = []
        recording = False
        file_count = 0

        self.get_logger().info("マイク起動")

        try:
            if self.flag_emosub:
                self.jtalk("こんにちは")
            while rclpy.ok():
                # マイク入力
                data = stream.read(CHUNK, exception_on_overflow=False)
                audio_chunk = np.frombuffer(data, dtype=np.int16)
                buffer.append(audio_chunk)

                # 音声データをVADで判定
                audio_tensor = torch.from_numpy(np.concatenate(buffer)).float() / 32768.0
                is_speech = get_speech_timestamps(audio_tensor, model, sampling_rate=RATE, threshold=0.3)

                if is_speech and not recording:
                    self.get_logger().info("発話開始検知")
                    recording = True
                    speech_audio = []  # 初期化
                
                if recording:
                    speech_audio.extend(audio_chunk)  # データ連結

                    if not is_speech:
                        self.get_logger().info("発話終了検知")
                        recording = False

                        file_count += 1
                        speech_audio = list(first_speech_audio) + speech_audio
                        #byte_data = pickle.dumps(speech_audio)
                        #send_audio = io.BytesIO(byte_data)
                        first_speech_audio.clear()  # 初期化
                        file_name = f"speech_{file_count}.wav"
                        output_path = os.path.join(output_dir, file_name)
                        write(output_path, RATE, np.array(speech_audio, dtype=np.int16))
                        self.get_logger().info(f"保存 {output_path}")
                        
                        file = {'file': (f"{file_name}", open(f"{output_path}", 'rb'), 'audio/wav')}
                        print(type(file))
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
                                
                                #play_audio(audio_data)
                                self.play_jtalk(voice, total_time)
                                #total_time += end_time - start_time
                                length = len(response.json()['text'])
                                #self.play_jtalk(voice, total_time)
                                #self.get_logger().info("talk time: {:.2f} seconds".format(end_time - start_time))
                            else:
                                self.get_logger().info(f"Error: {response.status_code}")
                        except Exception as e:
                            self.get_logger().info(f"Request failed: {e}")
                else:
                    first_speech_audio.extend(audio_chunk)

        except KeyboardInterrupt:
            pass
        finally:
            stream.stop_stream()
            stream.close()
            audio.terminate()


def main(args=None):
    try:
      rclpy.init(args=args)
      talker = TalkCtrl()
      talker.talk()
      rclpy.spin(talker)
    except KeyboardInterrupt:
      pass
    finally:
      talker.destroy_node()
      rclpy.shutdown()
  
if __name__ == '__main__':
    main()
