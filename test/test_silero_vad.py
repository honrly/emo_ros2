import threading
import numpy as np
import wave
import time
import torch
from collections import deque
import pyaudio
import sounddevice as sd

SAMPLE_RATE = 16000
CHANNELS = 1  # チャンネル1(モノラル), ReSpeaker 4 Mic Array = max 6だけど音声検知をリアルタイムで処理するにはラズパイだと馬鹿多いので1で良い
CHUNK_SIZE = 2048 # ラズパイは2048、1回のデータ取得量
BUFFER_SIZE = 10 # どれくらい貯めるか
SILENCE_DURATION = 1.0  # 発話終了と判定する無音時間

model = None
get_speech_timestamps = None

is_speaking = False
silence_start_time = None

buffer = deque(maxlen=BUFFER_SIZE)
recorded_data = []
lock = threading.Lock()
event = threading.Event()

audio = pyaudio.PyAudio()
fmt = pyaudio.paInt16

def load_model():
    # path = "/home/user/turtlebot3_ws/src/emotion_ros/model/snakers4-silero-vad-3f9fffc"
    # model_path = path + "/src/silero_vad/data/silero_vad.jit"
    # model, utils = torch.hub.load(path, 'custom', path=model_path, source='local')
    model, utils = torch.hub.load(repo_or_dir='snakers4/silero-vad', model='silero_vad', force_reload=False)
    (get_speech_timestamps, _, _, _, _) = utils
    return model, get_speech_timestamps

def save_wave(filename, data, sample_rate):
    with wave.open(filename, 'wb') as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(audio.get_sample_size(fmt))
        wf.setframerate(sample_rate)
        wf.writeframes(b''.join(data))

def audio_record():
    global recorded_data
    file_count = 0
    
    stream = audio.open(format=fmt, channels=CHANNELS, rate=SAMPLE_RATE, 
                        input=True, input_device_index=3,frames_per_buffer=CHUNK_SIZE)
    print("Thread A: マイク起動")
    try:
        while True:
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
                    save_wave(f"./output_audio/speech_{file_count}.wav", recorded_data, SAMPLE_RATE)
                    recorded_data.clear()
                    print(f"Thread A: 保存speech_{file_count}")
                event.clear()
                buffer.clear()
    finally:
        stream.stop_stream()
        stream.close()

def do_vad():
    global is_speaking, silence_start_time
    while True:
        time.sleep(0.1)  # チェック間隔
        with lock:
            if len(buffer) > 0:
                # 音声データを処理
                audio_tensor = torch.from_numpy(np.concatenate(buffer)).float() / 32768.0  # Normalize audio data

                # 発話のタイムスタンプ
                speech_timestamps = get_speech_timestamps(audio_tensor, model, sampling_rate=SAMPLE_RATE, threshold=0.3)

                if speech_timestamps:
                    # 発話開始
                    if not is_speaking:
                        print("Thread B: 発話開始")
                        for buf_data in buffer:
                            recorded_data.append(buf_data.tobytes())
                    is_speaking = True # 発話中
                    silence_start_time = None  # 無音時間リセット
                else:
                    # 無音が一定時間継続した場合に発話終了と判定
                    if is_speaking:
                        if silence_start_time is None:
                            silence_start_time = time.time()
                        elif time.time() - silence_start_time >= SILENCE_DURATION:
                            print("Thread B: 発話終了")
                            is_speaking = False
                            buffer.clear()
                            event.set()  # 発話終了のシグナルを送信
                    else:
                        silence_start_time = None  # 無音時間リセット

def main():
    global model, get_speech_timestamps
    model, get_speech_timestamps = load_model()

    thread_a = threading.Thread(target=audio_record, daemon=True)
    thread_b = threading.Thread(target=do_vad, daemon=True)

    thread_a.start()
    thread_b.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        audio.terminate()
        
if __name__ == '__main__':
    main()

