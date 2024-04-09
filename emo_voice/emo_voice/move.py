import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import numpy as np

MAX_LEN_DATA = 30
pnnx_rest_ave = -1
first_move = True # 最初のロボの行動

class MoveNode(Node):
    def __init__(self):
        super().__init__('move_node')
        
        self.create_subscription(Int32, 'pnnx', self.move_callback, 10)
        self.create_subscription(Int32, 'pnnx_rest_ave', self.pnnx_rest_callback, 10)

        self.pub_move = self.create_publisher(Twist, 'cmd_vel', 10)
        self.vel = Twist()

        self.pnnx_stimuli = [] # ロボが行動したときのpnnxデータ(30個)
        self.pnnx_stimuli_ave = -1 # ロボが行動したときのpnnx平均

    def move_callback(self, msg_serial_pnnx):
        global pnnx_rest_ave, first_move
        print(f"pnnx:{msg_serial_pnnx.data}, 安静時:{pnnx_rest_ave}")
        
        # ロボ行動時のpnnxデータを貯める
        if len(self.pnnx_stimuli) < MAX_LEN_DATA:
            if first_move:
                self.vel.angular.z = 0.3
                first_move = False
            self.pnnx_stimuli.append(msg_serial_pnnx.data)
        # ロボ行動時のpnnxデータが貯まっていれば平均を出す
        else:
            self.pnnx_stimuli_ave = np.mean(self.pnnx_stimuli)
            self.pnnx_stimuli.clear()

        # 安静時と直前のpnnxの平均を比較して動かす
        if self.pnnx_stimuli_ave > pnnx_rest_ave:
            # うれしい
            self.vel.angular.z = 0.5
        else:
            # 悲しい
            self.vel.angular.z = 0.1
        
        self.pub_move.publish(self.vel)
        
    def pnnx_rest_callback(self, msg_pnnx_rest):
        global pnn_rest_ave
        pnn_rest_ave = msg_pnnx_rest.data
    
    def run(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    listener = MoveNode()
    try:
        listener.run()
    except KeyboardInterrupt:
        pass
    finally:
        listener.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
