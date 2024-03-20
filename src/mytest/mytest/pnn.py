import rclpy
import serial
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import numpy as np

MAX_LEN_DATA = 30

class PnnNode(Node):
    def __init__(self):
        super().__init__('pnn_node')
        
        self.first_move = True # 最初のロボの行動
        
        self.pnnx_rest = [] # 何も無い時(安静時)のpnnxデータ(30個)
        self.pnnx_stimuli = [] # ロボが行動したときのpnnxデータ(30個)
        
        self.pnnx_stimuli_ave = -1 # ロボが行動したときのpnnx平均
        
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.vel = Twist()
        
        self.pub_pnnx = self.create_publisher(Int32, 'pnnx_plot', 10)
        self.pub_pnnx_rest_ave = self.create_publisher(Int32, 'pnnx_rest_ave', 10)
        
        self.pnnx_rest_ave = Int32() # 何も無い時のpnnx平均
        self.pnnx_rest_ave.data = -1 # 格納、送信データ
        
        self.create_subscription(Int32, 'serial_pnnx', self.move, 10)
    
    def pnnx_callback(self):
        # global PNNX
        # self.pnnx.data = PNNX
        # pnnxの値に応じて動かす
        self.move()
            
        # print(f"RRI: {self.RRI}, pnnx: {self.pnnx.data}\n")

    def move(self, msg_serial_pnn):
        print(f"pnnx:{msg_serial_pnn.data}")
        # 安静時のpnnxデータを貯める
        if len(self.pnnx_rest) < MAX_LEN_DATA:
            self.pnnx_rest.append(msg_serial_pnn.data)
        # 安静時データが貯まっていてまだ平均を出していなければ出す
        elif self.pnnx_rest_ave.data == -1:
            self.pnnx_rest_ave.data = int(np.mean(self.pnnx_rest))
            self.pub_pnnx_rest_ave.publish(self.pnnx_rest_ave)
        # ロボ行動時のpnnxデータを貯める
        elif len(self.pnnx_stimuli) < MAX_LEN_DATA:
            if self.first_move:
                self.vel.angular.z = 0.3
                self.first_move = False
            self.pnnx_stimuli.append(msg_serial_pnn.data)
        # ロボ行動時のpnnxデータが貯まっていれば平均を出す
        else:
            self.pnnx_stimuli_ave = np.mean(self.pnnx_stimuli)
            self.pnnx_stimuli.clear()

        # 安静時と直前のpnnxの平均を比較して動かす
        if self.pnnx_stimuli_ave > self.pnnx_rest_ave.data:
            # うれしい
            self.vel.angular.z = 0.5
        else:
            # 悲しい
            self.vel.angular.z = 0.1

        self.pub.publish(self.vel)
        int32_msg = Int32()
        int32_msg.data = msg_serial_pnn.data
        self.pub_pnnx.publish(int32_msg)
        
        
    def run(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    listener = PnnNode()
    try:
        listener.run()
    except KeyboardInterrupt:
        pass
    finally:
        listener.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
