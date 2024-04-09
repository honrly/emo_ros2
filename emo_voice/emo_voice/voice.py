import rclpy
from rclpy.node import Node
from emo_voice_interfaces.srv import GenText
from time import sleep

import requests
import json
import pygame
import google.generativeai as genai
import numpy as np

GEMINI_API_KEY = 'AIzaSyBngdp3djAPSK6MjNQm4U5TjJaACxJepvE'

def vvox_test(text):
    # エンジン起動時に表示されているIP、portを指定
    host = 'localhost'
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
        headers={'Content-Type': 'application/json'},
        params=params,
        data=json.dumps(query.json())
    )

    return synthesis.content   

class VoiceNode(Node):
    def __init__(self):
        super().__init__('voice_node')

        # Gemini初期設定
        genai.configure(api_key=GEMINI_API_KEY)
        self.model = genai.GenerativeModel('gemini-pro')
        
        self.srv = self.create_service(GenText, 'gen_text_srv', self.gen_text_callback)
        
    def gen_text_callback(self, request, response):
        self.get_logger.info(request.emo)
        if request.emo == 'Relax':
            text = f'ある人はリラックスしています。嬉しいという反応を1つ簡潔に返してください。'
        elif request.emo == 'Normal':
            text = f'短い挨拶を1つ返してください。'
        elif request.emo == 'Tense':
            text = f'ある人は緊張しています。その人をリラックスさせるような短い声掛けを1つ返してください。'
        
        self.get_logger.info(text)
        res_gemini = self.model.generate_content(text)
        self.get_logger.info(res_gemini.text)
        # self.pub_text.publish(response_gemini.text)

        # voice = vvox_test(request.name + res_gemini.text)
        voice = vvox_test(res_gemini.text)
        # self.get_logger.info(len(voice))
        voice_bytes = bytes(voice)
        # self.get_logger.info(len(voice_bytes))
        voice_int = np.frombuffer(voice_bytes, dtype=np.int32).tolist()
        # voice_int = [int(x) for x in voice]
        response.text = voice_int
        return response
    
    def run(self):
        while rclpy.ok():
            rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    talker = VoiceNode()
    try:
        talker.run()
    except KeyboardInterrupt:
        pass
    finally:
        talker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
