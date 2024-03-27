import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from emo_voice_interfaces.srv import GenText

import pygame
import time
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

        # クライアントの生成
        self.cli = self.create_client(GenText, 'gen_text_srv')
        # サーバー接続まで待機
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available...")
        # リクエストの生成
        self.req = GenText.Request()

        self.create_subscription(Int32, 'sensor_pnnx', self.emo_callback, 10)
        # self.flg = False
        timer_period = 15.0
        self.timer = self.create_timer(timer_period, self.send_request)

        # 音声再生の初期化
        pygame.mixer.pre_init(frequency=24000, size=-16, channels=1)
        pygame.init()

    def emo_callback(self, msg):
        # self.flg = True
        pnnx = msg.data
        global status
        if pnnx < 20:
            status = '緊張'
        elif pnnx < 50:
            status = '普通'
        else:
            status = 'リラックス'

    def send_request(self):
        global status, name
        self.req.emo = status
        self.req.name = name + 'さん、'
        self.future = self.cli.call_async(self.req)
        print(f'req.emo:{status}')
        
    def run(self):
        # レスポンスの受信
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future.done():
                try:
                    response = self.future.result()
                    # print(len(response.text))
                    voicetext = response.text
                    voice = bytes(voicetext)
                    print(len(voice))
                    play(voice)
                    print('ok')
                except Exception as e:
                    # Pygameを終了
                    pygame.mixer.quit()
                    pygame.quit()
                    break

def main(args=None):
    rclpy.init(args=args)
    listener = ClientNode()
    listener.send_request()
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
