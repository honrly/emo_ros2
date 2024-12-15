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

GPU_SERVER_URL = 'http://192.168.65.146:5000/llm'

def play_audio(audio_data):
    tmp = pygame.mixer.Sound(buffer=audio_data)
    pygame.mixer.Sound.play(tmp)
    # 再生が終わるまで待機
    while pygame.mixer.get_busy():
        pygame.time.Clock().tick(10)

class TalkCtrl(Node):
    def __init__(self):
        super().__init__('talk_ctrl')
        
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
        threading.Thread(target=self.send_msg_server, daemon=True).start()
        
    def send_msg_server(self):
        if self.flag_emosub:
            self.jtalk("こんにちは")
        while rclpy.ok():
            audio_buffer = self.record_audio()  # 録音した音声データを取得
            print(type(audio_buffer))
            
            # バイト列データを直接送信
            files = {'file': ('usre_input.wav', audio_buffer, 'audio/wav')}
            emo = {'emo': f'{self.emo}'}
            self.get_logger().info(f"Send_emotion: {self.emo}")

            try:
                start_time = time.time()
                # 音声データとその他のデータをFlaskアプリに送信
                response = requests.post(GPU_SERVER_URL, files=files, data=emo)
                if response.status_code == 200:
                    #end_time = time.time()
                    #self.get_logger().info("総時間: {:.2f} seconds, {}文字".format(end_time - start_time, len(response.json()['response'])))
                    
                    #start_time = time.time()
                    #self.get_logger().info(f"Your Voice: {response.json()['input']}")
                    #self.get_logger().info(f"Response: {response.json()['response']}")

                    #self.jtalk(f"{response.json()['response']}")
                    end_time = time.time()
                    voice = base64.b64decode(response.json()['voice'])
                    
                    self.get_logger().info(f"Your Voice: {response.json()['input']}")
                    self.get_logger().info(f"Received from Flask: {response.json()['text']}")
                    self.get_logger().info("生成時間: {:.2f} seconds, {}文字".format(end_time - start_time, len(response.json()['text'])))
                    total_time = end_time - start_time
                    
                    #start_time = time.time()
                    #voice_response = requests.post(CPU_SERVER_URL, json={"text": text})
                    #audio_data = base64.b64decode(voice_response.json()['voice'])
                    
                    #audio_data = base64.b64decode(f"{response.json()['voice']}")
                    #end_time = time.time()
                    #self.get_logger().info("合成時間: {:.2f} seconds, {}文字".format(end_time - start_time, len(response.json()['text'])))
                    
                    
                    #play_audio(audio_data)
                    self.play_jtalk(voice, total_time)
                    #total_time += end_time - start_time
                    length = len(response.json()['text'])
                    #self.play_jtalk(voice, total_time)
                    #self.get_logger().info("talk time: {:.2f} seconds".format(end_time - start_time))
                    
                    #self.engine.runAndWait()
                else:
                    self.get_logger().info(f"Error: {response.status_code}")
            except Exception as e:
                self.get_logger().info(f"Request failed: {e}")
      
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
    
    def record_audio(self):
        rec_time = 20             # 録音時間[s]
        fmt = pyaudio.paInt16     # 音声のフォーマット
        ch = 6                    # チャンネル1(モノラル), ReSpeaker 4 Mic Array = max 6
        sampling_rate = 16000     # サンプリング周波数
        chunk = 2048             # チャンク（データ点数）, 2048
        audio = pyaudio.PyAudio()
        index = 3                 # 録音デバイスのインデックス番号（デフォルト1）

        # 録音ストリームの初期化
        stream = audio.open(format=fmt, channels=ch, rate=sampling_rate, input=True,
                            input_device_index=index,
                            frames_per_buffer=chunk)
        self.get_logger().info("Recording start...")

        # 録音処理
        frames = []
        for i in range(0, int(sampling_rate / chunk * rec_time)):
            data = stream.read(chunk)
            frames.append(data)

        self.get_logger().info("Recording end...")

        # 録音ストリームの終了処理
        stream.stop_stream()
        stream.close()
        audio.terminate()

        # バッファに録音データを保存
        audio_buffer = io.BytesIO()
        with wave.open(audio_buffer, 'wb') as wav:
            wav.setnchannels(ch)
            wav.setsampwidth(audio.get_sample_size(fmt))
            wav.setframerate(sampling_rate)
            wav.writeframes(b''.join(frames))

        audio_buffer.seek(0)  # バッファの先頭に移動
        return audio_buffer


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
