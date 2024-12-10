
import os
import time
import torch
import torchaudio
import io
import subprocess
import base64
import numpy as np

import re
import unicodedata

from alkana import get_kana
import markdown
from bs4 import BeautifulSoup

import pyaudio
import pygame
import wave

cmd = ''

def init_jtalk():
    global cmd
    open_jtalk = ['open_jtalk']
    mech = ['-x', '/var/lib/mecab/dic/open-jtalk/naist-jdic']
    htsvoice = ['-m', '/usr/share/hts-voice/mei/mei_normal.htsvoice']
    speed = ['-r', '1.0']
    outwav = ['-ow', '/dev/stdout']
    cmd = open_jtalk + mech + htsvoice + speed + outwav
    pygame.init()

init_jtalk()

def jtalk_bytes(t):
    start_time = time.time()  # 処理開始時間
    
    # Open JTalkで音声を生成（標準出力に書き出し）
    c = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    audio_data, _ = c.communicate(input=t.encode('utf-8'))  # 直接データを渡す
    
    end_time = time.time()  # 処理終了時間
    print("合成時間: {:.2f} seconds".format(end_time - start_time))
    
    return audio_data

# ref https://stackoverflow.com/questions/761824/python-how-to-convert-markdown-formatted-text-to-text
def md_to_text(md):
    html = markdown.markdown(md)
    soup = BeautifulSoup(html, features='html.parser')
    return soup.get_text()
    
    
def is_japanese(ch):
    name = unicodedata.name(ch, None)
    return name is not None and \
            ("CJK UNIFIED" in name or "HIRAGANA" in name or "KATAKANA" in name)


def includes_japanese(s):
    return any(is_japanese(ch) for ch in s)


def convert_english_words(L):
    ws = re.split(r"([a-zA-Z']+)", L)
    ys = []
    for w in ws:
        if re.fullmatch("[a-zA-Z']+", w):
            w = get_kana(w) or get_kana(w.lower()) or w
        ys.append(w)
    return ''.join(ys)


def parse_lines(text):
    text = re.sub(r'\n\s*(?<=[a-zA-Z])', '。', text)
    text = re.sub(r'\n\s*(?<=[^a-zA-Z])', '。', text)
    text = text.replace('\n\n', '。')

    # remove spaces between zenkaku and hankaku
    text = re.sub(r'((?!\n)\s)+', ' ', text)
    s = list(text)
    for i in range(1, len(text) - 1):
        prev_ch, ch, next_ch = s[i-1], s[i], s[i+1]
        if ch == ' ':
            prev_ch_is_japanese = is_japanese(prev_ch)
            next_ch_is_japanese = is_japanese(next_ch)
            if prev_ch_is_japanese and not next_ch_is_japanese or not prev_ch_is_japanese and next_ch_is_japanese:
                s[i] = ''
    text = ''.join(s)

    lines = re.split(r"([。\n]+)", text)
    r = []
    for L in lines:
        if not L:
            pass
        elif re.fullmatch(r"[。\n]+", L):
            if r and "。" in L:
                r[-1] += "。"
        else:
            r.append(L)
    lines = r

    return lines

def play_jtalk(audio_data):
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
    
    tmp.play()

    # 再生が終わるまで待機
    while pygame.mixer.get_busy():
        pygame.time.Clock().tick(10)

def main():
    input_file = "test_parse.txt"
    with open(input_file) as inp:
        text = inp.read()
    text = md_to_text(text)
    print(text)
    lines = parse_lines(text)
    print(lines)
    #lines = [L for L in lines if includes_japanese(L)]
    #print(lines)
    lines = [convert_english_words(L) for L in lines]
    lines = "".join(lines)
    print(lines)

    voice = jtalk_bytes(lines)
    play_jtalk(voice)

if __name__ == '__main__':
    main()
