import requests
import json
import pygame
import io
import google.generativeai as genai
import random

GEMINI_API_KEY = 'AIzaSyBngdp3djAPSK6MjNQm4U5TjJaACxJepvE'

def vvox_test(text):
    # エンジン起動時に表示されているIP、portを指定
    host = "127.0.0.1"
    port = 50021
    
    # 音声化する文言と話者を指定(3で標準ずんだもんになる)
    params = {
        'text': text,
        'speaker': 3
    }
    
    # 音声合成用のクエリ作成
    query = requests.post(
        f'http://{host}:{port}/audio_query',
        params=params
    )
    
    # 音声合成を実施
    synthesis = requests.post(
        f'http://{host}:{port}/synthesis',
        headers={"Content-Type": "application/json"},
        params=params,
        data=json.dumps(query.json())
    )
    
    # Pygameを初期化
    pygame.init()
    pygame.mixer.init(frequency=24000)

    # 音声を再生
    voice = io.BytesIO(synthesis.content)
    pygame.mixer.music.load(voice)
    pygame.mixer.music.play()

    # 再生が終わるまで待機
    while pygame.mixer.music.get_busy():
        pygame.time.Clock().tick(10)

    # Pygameを終了
    pygame.mixer.quit()
    pygame.quit()
    
def gen_text_gemini(emo):
        if emo == 'リラックス':
            text = f'ある人は{emo}しています。一緒に嬉しくなる反応を1つ簡潔に返してください。'
        elif emo == '普通':
            text = f'短い挨拶を1つ返してください。'
        elif emo == '緊張':
            text = f'ある人は{emo}しています。その人をリラックスさせるような短い声掛けを1つ返してください。'
        
        print(text)
        genai.configure(api_key=GEMINI_API_KEY)
        model = genai.GenerativeModel('gemini-pro')
        response = model.generate_content(text)
        print(response.text)
        return response.text

if __name__ == "__main__":
    random_val = random.randint(0, 2)
    if random_val == 0:
        emo = 'リラックス'
    elif random_val == 1:
        emo = '普通'
    elif random_val == 2:
        emo = '緊張'
    print(emo)
    emo = 'リラックス'
    text = gen_text_gemini(emo)
    vvox_test(text)
