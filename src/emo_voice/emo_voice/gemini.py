import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import google.generativeai as genai

GEMINI_API_KEY = 'AIzaSyBngdp3djAPSK6MjNQm4U5TjJaACxJepvE'

class GeminiNode(Node):
    def __init__(self):
        super().__init__('gemini_node')
        
        genai.configure(api_key=GEMINI_API_KEY)
        self.model = genai.GenerativeModel('gemini-pro')
        # self.srv = self.create_service(Emo, 'gen_text_gemini_emo', self.listener_callback)
    
    def gen_text_gemini(self, emo):
        if emo == 'リラックス':
            text = f'ある人は{emo}しています。嬉しいという反応を1つ簡潔に返してください。'
        elif emo == '普通':
            text = f'短い挨拶を1つ返してください。'
        elif emo == '緊張':
            text = f'ある人は{emo}しています。その人をリラックスさせるような短い声掛けを1つ返してください。'
        
        print(text)
        response = self.model.generate_content(text)
        return response.text
        
    def run(self):
        while rclpy.ok():
            self.calc_pnnx()

def main(args=None):
    rclpy.init(args=args)
    talker = GeminiNode()
    try:
        talker.run()
    except KeyboardInterrupt:
        pass
    finally:
        talker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
