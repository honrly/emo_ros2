import subprocess
import concurrent.futures
import pygame
import wave
import io
import numpy as np

def synthesize_speech(text):
    """テキストを音声合成し、バイト列で返す"""
    # Open JTalkのコマンド設定
    openjtalk_command = [
        "open_jtalk",
        "-x", "/var/lib/mecab/dic/open-jtalk/naist-jdic",  # 辞書のパス
        "-m", "/usr/share/hts-voice/mei/mei_normal.htsvoice",        # 音声モデル
        "-ow", "/dev/stdout"                                     # 標準出力にWAV出力
    ]
    
    process = subprocess.Popen(openjtalk_command, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    stdout_data, _ = process.communicate(input=text.encode("utf-8"))
    return stdout_data


def combine_audio(audio_bytes_list):
    """複数の音声データを結合"""
    output_buffer = io.BytesIO()
    
    # 最初の音声データをベースにヘッダーを取得
    with wave.open(io.BytesIO(audio_bytes_list[0]), 'rb') as first_wave:
        params = first_wave.getparams()
    
    # 出力バッファに結合
    with wave.open(output_buffer, 'wb') as output_wave:
        output_wave.setparams(params)
        for audio_bytes in audio_bytes_list:
            with wave.open(io.BytesIO(audio_bytes), 'rb') as wave_file:
                output_wave.writeframes(wave_file.readframes(wave_file.getnframes()))
    
    return output_buffer.getvalue()

def play_jtalk(audio_data):
        pygame.mixer.quit()  # ミキサーを初期化し直す
        print("point")

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
        
        tmp.play()

        # 再生が終わるまで待機
        while pygame.mixer.get_busy():
            pygame.time.Clock().tick(10)
            
def main():
    # 音声合成するテキスト
    texts = ["こんにちは、これはテストです。", "この文章が次に続きます。"]
    
    # 並列で音声合成を実行
    with concurrent.futures.ThreadPoolExecutor() as executor:
        audio_futures = [executor.submit(synthesize_speech, text) for text in texts]
        audio_results = [future.result() for future in concurrent.futures.as_completed(audio_futures)]
    
    # 音声データを結合
    combined_audio = combine_audio(audio_results)
    print(type(combined_audio))
    
    # pygameで再生
    pygame.mixer.init()
    play_jtalk(combined_audio)
    
    # 再生が終了するまで待機
    while pygame.mixer.get_busy():
        pygame.time.wait(10)


if __name__ == "__main__":
    main()
