import rclpy
from rclpy.node import Node
from emo_voice_interfaces.srv import GenText

import requests
import json
import pygame
import io
import google.generativeai as genai
import numpy as np

GEMINI_API_KEY = 'AIzaSyBngdp3djAPSK6MjNQm4U5TjJaACxJepvE'

def vvox_test(text):
    # エンジン起動時に表示されているIP、portを指定
    host = "192.168.0.8"
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

    return synthesis.content
    

class VoiceNode(Node):
    def __init__(self):
        super().__init__('voice_node')

        # Gemini初期設定
        genai.configure(api_key=GEMINI_API_KEY)
        self.model = genai.GenerativeModel('gemini-pro')
        
        self.srv = self.create_service(GenText, 'gen_text_srv', self.gen_text_callback)
        
    def gen_text_callback(self, request, response):
        if request.emo == 'リラックス':
            text = f'ある人は{request.emo}しています。嬉しいという反応を1つ簡潔に返してください。'
        elif request.emo == '普通':
            text = f'短い挨拶を1つ返してください。'
        elif request.emo == '緊張':
            text = f'ある人は{request.emo}しています。その人をリラックスさせるような短い声掛けを1つ返してください。'
        
        print(text)
        response_gemini = self.model.generate_content(text)
        print(response_gemini.text)
        voice = vvox_test(request.name + response_gemini.text)
        print("bytes")
        voice_bytes = bytes(voice)
        voice_int = np.frombuffer(voice_bytes, dtype=np.int32).tolist()

        # Assign the list of integers to response.text
        response.text = voice_int
        return response
        # print(type(voice))
        # print(len(voice))
        # self.pub_text.publish(response_gemini.text)
        # values = [int(x) for x in voice]
        # print("int")
        # print(len(values))
        # print(type(values[0]))
        # response.text = voice
        # print(len(response.text))
        # return response
    
    def run(self):
        while rclpy.ok():
            rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    talker = VoiceNode()
    print("a")
    try:
        talker.run()
    except KeyboardInterrupt:
        pass
    finally:
        talker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
