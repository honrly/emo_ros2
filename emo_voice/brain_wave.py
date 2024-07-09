import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from my_custom_message.msg import BrainData
from neuropy3.neuropy3 import MindWave
from time import sleep
import datetime

MINDWAVE_ADDRESS = 'C4:64:E3:E7:C6:71'

class BrainWaveNode(Node):
    def __init__(self):
        super().__init__('brain_wave_node')

        # MindWave Setting
        self.mw = MindWave(address=MINDWAVE_ADDRESS, autostart=False, verbose=3)
        # self.mw = MindWave(autostart=False, verbose=3)  # Autoscan for MindWave Mobile
        self.mw.set_callback('signal', self.publish_brain_wave) # poor signal
        self.mw.set_callback('eeg', self.set_brain_data)
        self.mw.start()

        # Publish MindWave Data
        self.pub_brain_wave = self.create_publisher(BrainData, 'brain_wave', 10)
        self.brain_wave = BrainData()
        self.brain_wave.poorsignal = 0
        self.brain_wave.delta = 0
        self.brain_wave.theta = 0
        self.brain_wave.alpha_l = 0
        self.brain_wave.alpha_h= 0
        self.brain_wave.beta_l = 0
        self.brain_wave.beta_h = 0
        self.brain_wave.gamma_l = 0
        self.brain_wave.gamma_m = 0

        #timer_period = 1.0
        #self.timer = self.create_timer(timer_period, self.publish_)

    def set_brain_data(self, eeg_data):
        self.brain_wave.delta = eeg_data['delta']
        self.brain_wave.theta = eeg_data['theta']
        self.brain_wave.alpha_l = eeg_data['alpha_l']
        self.brain_wave.alpha_h= eeg_data['alpha_h']
        self.brain_wave.beta_l = eeg_data['beta_l']
        self.brain_wave.beta_h = eeg_data['beta_h']
        self.brain_wave.gamma_l = eeg_data['gamma_l']
        self.brain_wave.gamma_m = eeg_data['gamma_m']
        
        self.get_logger().info(f'EED_DATA: {eeg_data}\n\n')

    def publish_brain_wave(self, poorsignal_data):
        self.brain_wave.poorsignal = poorsignal_data
        self.get_logger().info(f'POORSIGNAL: {self.brain_wave.poorsignal}')
        self.pub_brain_wave.publish(self.brain_wave)
    
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
    except TimeoutError:
        pass
    finally:
        talker.mw.unset_callback('eeg')
        talker.mw.unset_callback('signal')
        talker.mw.stop()
        talker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
