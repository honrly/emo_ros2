import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, String

MAX_LEN_DATA = 30
X_PNN = 50

class PnnxNode(Node):
    def __init__(self):
        super().__init__('emo_status_node')
        
        self.create_subscription(Int32, 'pnnx', self.pnnx_callback, 10)
        self.create_subscription(Float32, 'brain_wave', self.brain_wave_callback, 10)
        self.pnnx = 0
        self.b_a = 0

        self.pub_emo_status = self.create_publisher(String, 'emo_status', 10)
        self.emo_status = String() # 感情状態
        self.emo_status.data = '' # 送信データ

    def pnnx_callback(self, msg):
        self.pnnx = msg.data
        self.get_logger.info(f'pnnx:{self.pnnx}')
    
    def brain_wave_callback(self, msg):
        self.b_a = msg.data
        self.get_logger.info(f'lowB/lowA:{self.b_a}')

    def publish_emo_status(self):
        if self.pnnx < 20 and self.b_a > 1.0:
            self.emo_status.data = 'Tense'
        elif self.pnnx < 50 and self.b_a > 0.5:
            self.emo_status.data = 'Normal'
        else:
            self.emo_status.data = 'Relax'
        
        self.pub_emo_status.publish(self.emo_status)
    
    def run(self):
        while rclpy.ok():
            self.publish_pnnx()

def main(args=None):
    rclpy.init(args=args)
    talker = PnnxNode()
    try:
        talker.run()
    except KeyboardInterrupt:
        pass
    finally:
        talker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
