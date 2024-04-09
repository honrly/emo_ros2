import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, String
from emo_voice_interfaces.srv import GenText

import pygame
from time import sleep
import io

status = 'Relax'
name = 'ずんだ'

def play(voice):
    tmp = pygame.mixer.Sound(buffer=voice)
    pygame.mixer.Sound.play(tmp)
    # 再生が終わるまで待機
    while pygame.mixer.get_busy():
        pygame.time.Clock().tick(10)

class ClientNode(Node):
    def __init__(self):
        super().__init__('client_node')

        self.is_request_sent = False
        # クライアントの生成
        self.cli = self.create_client(GenText, 'gen_text_srv')
        # サーバー接続まで待機
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available...')
        # リクエストの生成
        self.req = GenText.Request()

        self.emo_status = ''
        self.create_subscription(String, 'emo_status', self.emo_status_callback, 10)
        
        timer_period = 15.0
        self.timer = self.create_timer(timer_period, self.send_request)

        # 音声再生の初期化
        pygame.mixer.pre_init(frequency=24000, size=-16, channels=1)
        pygame.init()

    def emo_status_callback(self, msg):
        self.emo_status = msg.data
        self.get_logger.info(f'emo_status:{self.emo_status}')
    
    def send_request(self):
        global status, name
        if not self.is_request_sent:
            self.req.emo = status
            self.req.name = name + 'さん、'
            self.future = self.cli.call_async(self.req)
            self.get_logger.info(f'req.emo:{status}')
            self.is_request_sent = True
        
    def run(self):
        # レスポンスの受信
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.is_request_sent and self.future.done():
                try:
                    response = self.future.result()
                    voicetext = response.text
                    voice = bytes(voicetext)
                    # self.get_logger.info(len(voice))
                    play(voice)
                    # self.get_logger.info('ok')
                    self.is_request_sent = False
                except Exception as e:
                    self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    listener = ClientNode()
    try:
        listener.run()
    except KeyboardInterrupt:
        pass
    finally:
        # Pygameを終了
        pygame.mixer.quit()
        pygame.quit()

        listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
