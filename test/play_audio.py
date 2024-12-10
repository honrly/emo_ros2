#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pygame.mixer
import time
import os

# mixerモジュールの初期化
pygame.mixer.init()

# ディレクトリのパス
directory = "output_audio"

# ディレクトリ内のすべてのファイルを取得し、.wavファイルのみを抽出
wav_files = [f for f in os.listdir(directory) if f.endswith(".wav")]

# ファイルをソート（必要に応じて並び替えを調整）
wav_files.sort()

# 各wavファイルを再生
for wav_file in wav_files:
    file_path = os.path.join(directory, wav_file)
    print(f"再生中: {file_path}")
    sound = pygame.mixer.Sound(file_path)
    
    # 音楽再生（1回のみ）
    sound.play()
    
    # 再生が終わるまで待機
    time.sleep(sound.get_length())
    
    # 再生の終了
    sound.stop()

print("すべてのファイルを再生しました。")
