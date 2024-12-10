import pyaudio
import sounddevice as sd

print(sd.query_devices())

# Initialize PyAudio
audio = pyaudio.PyAudio()

# List all available audio devices
for i in range(audio.get_device_count()):
    info = audio.get_device_info_by_index(i)
    print(f"デバイスID: {i}, デバイス名: {info['name']}, チャンネル数: {info['maxInputChannels']}")

# Get the default sample rate of the first device
default_sample_rate = audio.get_device_info_by_index(0)['defaultSampleRate']
print(f"サポートされるサンプリングレート: {default_sample_rate}")

fmt = pyaudio.paInt16
stream = audio.open(format=fmt, channels=1, rate=16000, input=True,
                            input_device_index=3,
                            frames_per_buffer=2048)
# Terminate PyAudio
stream.stop_stream()
stream.close()
audio.terminate()
