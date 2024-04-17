import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from emo_voice_interfaces.srv import GenText

import google.generativeai as genai

GEMINI_API_KEY = 'AIzaSyBngdp3djAPSK6MjNQm4U5TjJaACxJepvE'

class GenTextNode(Node):
    def __init__(self):
        super().__init__('gemini_node')
        
        # Gemini初期設定
        genai.configure(api_key=GEMINI_API_KEY)
        self.model = genai.GenerativeModel('gemini-pro')
        # サービス
        self.srv = self.create_service(GenText, 'gen_text_srv', self.gen_text_callback)
        self.pub_text = self.create_publisher(String, 'gen_text', 10)
    
    def gen_text_callback(self, request):
        if request.emo == 'Relax':
            text = f'ある人は{request.emo}しています。嬉しいという反応を1つ簡潔に返してください。'
        elif request.emo == 'Normal':
            text = f'短い挨拶を1つ返してください。'
        elif request.emo == 'Tense':
            text = f'ある人は{request.emo}しています。その人をリラックスさせるような短い声掛けを1つ返してください。'
        
        self.get_logger().info(text)
        response_gemini = self.model.generate_content(text)
        self.pub_text.publish(response_gemini.text)
        # response.text = response_gemini.text
        # return response
        
    def run(self):
        while rclpy.ok():
            self.gen_text_callback()

def main(args=None):
    rclpy.init(args=args)
    talker = GenTextNode()
    try:
        talker.run()
    except KeyboardInterrupt:
        pass
    finally:
        talker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
