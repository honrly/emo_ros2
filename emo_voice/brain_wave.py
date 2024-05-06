import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from neuropy3.neuropy3 import MindWave
from time import sleep
import datetime

MINDWAVE_ADDRESS = 'C4:64:E3:E7:C6:71'

class BrainWaveNode(Node):
    def __init__(self):
        super().__init__('brain_wave_node')

        # MindWave Setting
        self.mw = MindWave(address=MINDWAVE_ADDRESS, autostart=False, verbose=3, retry_count=3)
        # self.mw = MindWave(autostart=False, verbose=3, retry_count=3)  # Autoscan for MindWave Mobile
        self.mw.set_callback('attention', self.get_att)
        self.mw.set_callback('meditation', self.get_med)
        self.mw.start()

        # Publish Attention-Meditation
        self.pub_att_med = self.create_publisher(Int32, 'brain_wave', 10)
        self.att_med = Int32()
        self.att_med.data = 0
        self.attention_val = 0
        self.meditation_val = 0

        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.publish_att_med)

    def get_att(self, mw_data):
        self.attention_val = mw_data

    def get_med(self, mw_data):
        self.meditation_val = mw_data
    
    def publish_att_med(self):
        self.att_med.data = self.attention_val - self.meditation_val
        self.pub_att_med.publish(self.att_med)
        self.get_logger().info(f'Att - Med: {self.att_med.data}, {self.attention_val}, {self.meditation_val}')
    
    def run(self):
        while rclpy.ok():
            rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    talker = BrainWaveNode()
    try:
        talker.run()
    except KeyboardInterrupt:
        pass
    finally:
        talker.mw.unset_callback('attention')
        talker.mw.unset_callback('meditation')
        talker.mw.stop()
        talker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
