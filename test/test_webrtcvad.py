import sounddevice as sd
import webrtcvad
import numpy as np
import queue
import collections
import threading
import wave
import datetime
import os

import openai
from google.cloud import texttospeech
import simpleaudio
import os
import speech_recognition as sr
import wave

SAMPLE_RATE = 48000
FRAME_DURATION = 20  # ms
FRAME_SIZE = int(SAMPLE_RATE * FRAME_DURATION / 1000)
CHANNELS = 1
PRE_ROLL_FRAMES = 10

vad = webrtcvad.Vad(3)

q = queue.Queue()
pre_roll_buffer = collections.deque(maxlen=PRE_ROLL_FRAMES)

# 音声記録用バッファ
recording_buffer = []
is_recording = False
lock = threading.Lock()


def callback(indata, frames, time, status):
    if status:
        print("ステータス:", status)

    audio = indata[:, 0]
    audio_bytes = (audio * 32767).astype(np.int16).tobytes()
    q.put(audio_bytes)


def save_wav(filename, audio_bytes):
    with wave.open(filename, 'wb') as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(2)  # 16-bit = 2 bytes
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(b''.join(audio_bytes))
    print(f"WAVファイル保存: {filename}")


def vad_worker():
    global is_recording, recording_buffer
    is_speech_prev = False
    silence_counter = 0
    silence_threshold = int(1000 / FRAME_DURATION)  # e.g., 20ms → 50 frames
    
    r = sr.Recognizer()

    while True:
        frame = q.get()
        if len(frame) != FRAME_SIZE * 2:
            continue

        pre_roll_buffer.append(frame)
        is_speech = vad.is_speech(frame, SAMPLE_RATE)

        # 発話開始（無音 → 音声）
        if is_speech and not is_speech_prev:
            if not is_recording:
                print("音声検出（録音開始）")
                with lock:
                    is_recording = True
                    recording_buffer = list(pre_roll_buffer)
            silence_counter = 0

        # 発話中なら録音続行
        if is_recording:
            with lock:
                recording_buffer.append(frame)

        # 無音が続いている場合
        if not is_speech:
            silence_counter += 1
        else:
            silence_counter = 0

        # 発話終了（無音が1秒続いたとき）
        if is_recording and silence_counter >= silence_threshold:
            print("無音（1秒継続）→ 録音終了\n")
            silence_counter = 0
            with lock:
                is_recording = False
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                os.makedirs("recordings", exist_ok=True)
                save_wav(f"recordings/voice_{timestamp}.wav", recording_buffer)
                
                #raw_data = b''.join(recording_buffer)  # bytes に変換
                #audio_data = sr.AudioData(raw_data, SAMPLE_RATE, 2) 
                
                # WAVとして保存
                with wave.open("temp.wav", "wb") as wf:
                    wf.setnchannels(1)
                    wf.setsampwidth(2)  # 16bit
                    wf.setframerate(SAMPLE_RATE)
                    wf.writeframes(b''.join(recording_buffer))

                # 認識処理
                with sr.AudioFile("temp.wav") as source:
                    audio = r.record(source)
                    cmd = r.recognize_google(audio, language="en-US")
                
                
                #cmd = r.recognize_google(audio_data, language='en-US')
                print("You: {}".format(cmd))
                recording_buffer = []

        is_speech_prev = is_speech



# VAD判定を別スレッドで実行
threading.Thread(target=vad_worker, daemon=True).start()

# マイクからの入力ストリーム開始
with sd.InputStream(
    channels=CHANNELS,
    samplerate=SAMPLE_RATE,
    blocksize=FRAME_SIZE,
    dtype='float32',
    callback=callback
):
    print("録音開始（Ctrl+Cで停止）")
    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("\n停止します。")
