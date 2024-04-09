import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, String
from emo_voice_interfaces.srv import GenText

import pygame
from time import sleep
import io

status = '緊張'
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

        self.pnnx = 0
        self.b_a = 0.0
        self.is_request_sent = False

        # クライアントの生成
        self.cli = self.create_client(GenText, 'gen_text_srv')
        # サーバー接続まで待機
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available...")
        # リクエストの生成
        self.req = GenText.Request()

        self.create_subscription(Int32, 'pnnx', self.pnnx_callback, 10)
        self.create_subscription(Float32, 'brain_wave', self.brain_wave_callback, 10)
        timer_period_emo = 1.0
        timer_period = 15.0
        self.timer_emo = self.create_timer(timer_period_emo, self.emo_callback)
        self.timer = self.create_timer(timer_period, self.send_request)

        # 音声再生の初期化
        pygame.mixer.pre_init(frequency=24000, size=-16, channels=1)
        pygame.init()

    def emo_callback(self):
        global status
        if self.pnnx < 20 and self.b_a > 1.0:
            status = '緊張'
        elif self.pnnx < 50 and self.b_a > 0.5:
            status = '普通'
        else:
            status = 'リラックス'
    
    def pnnx_callback(self, msg):
        self.pnnx = msg.data
        print(f'pnnx:{self.pnnx}')
    
    def brain_wave_callback(self, msg):
        self.b_a = msg.data
        print(f'lowB/lowA:{self.b_a}')

    def send_request(self):
        global status, name
        if not self.is_request_sent:
            self.req.emo = status
            self.req.name = name + 'さん、'
            self.future = self.cli.call_async(self.req)
            print(f'req.emo:{status}')
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
                    # print(len(voice))
                    play(voice)
                    # print('ok')
                    self.is_request_sent = False
                except Exception as e:
                    self.get_logger().error(f"Service call failed: {e}")

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
