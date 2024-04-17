import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from neuropy3.neuropy3 import MindWave
from time import sleep

MINDWAVE_ADDRESS = 'C4:64:E3:E7:C6:71'

class BrainWaveNode(Node):
    def __init__(self):
        super().__init__('brain_wave_node')

        # MindWave Setting
        self.mw = MindWave(address=MINDWAVE_ADDRESS, autostart=False, verbose=3, retry_count=3)
        # self.mw = MindWave(autostart=False, verbose=3, retry_count=3)  # Autoscan for MindWave Mobile
        self.mw.set_callback('eeg', self.pub_lowb_lowa)
        self.mw.start()

        # Publish lowBeta/lowAlpha
        self.publisher = self.create_publisher(Float32, 'brain_wave', 10)
        self.lowb_lowa = Float32()
        self.lowb_lowa.data = 0.0

    def pub_lowb_lowa(self, eeg_data):
        alpha_l = eeg_data['alpha_l']
        beta_l = eeg_data['beta_l']
        print("Alpha low:", alpha_l)
        print("Beta low:", beta_l)

        self.lowb_lowa.data = beta_l / alpha_l
        self.get_logger().info(f'Received lowb/lowa:{self.lowb_lowa.data}')
        self.publisher.publish(self.lowb_lowa)
    
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
        talker.mw.unset_callback('eeg')
        talker.mw.stop()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
