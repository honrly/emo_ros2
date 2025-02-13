#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import os

# フレームバッファの設定（fbset で確認した値）
FB_WIDTH = 592
FB_HEIGHT = 448

def display_jpg(image_path):
    # 画像を読み込み
    img = cv2.imread(image_path, cv2.IMREAD_COLOR)

    # フレームバッファのサイズにリサイズ
    img_resized = cv2.resize(img, (FB_WIDTH, FB_HEIGHT))

    # 画像データをRGB565（16bpp）に変換
    img_rgb565 = cv2.cvtColor(img_resized, cv2.COLOR_BGR2BGR565)
    img_bgra = cv2.cvtColor(img_resized, cv2.COLOR_BGR2BGRA)  # 32bpp に変換


    # フレームバッファに書き込む
    with open("/dev/fb0", "wb") as fb:
        fb.write(img_bgra.tobytes())

# 表示する JPG ファイルを指定
image_path = os.path.expanduser("~/turtlebot3_ws/src/emotion_ros/face_DB/anger5.JPG")

# 画像を表示
os.system("setterm -cursor off>/dev/tty0")
display_jpg(image_path)
