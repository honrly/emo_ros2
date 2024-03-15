import rclpy
import serial
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import numpy as np

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        
        self.ser = serial.Serial('/dev/ttyACM0', 115200)
        self.RRI = 0
        self.NNxCount = 0 # RRIの差が大きい瞬間数        
        self.x = 50 # pNNx
        self.df = [] # 計算対象データ(30個想定)
        self.pNNx_rest = [] # 何も無い時(安静時)のpNNxデータ(30個)
        self.pNNx_stimuli = [] # ロボが行動したときのpNNxデータ(30個)
        
        self.pNNx_stimuli_ave = -1 # ロボが行動したときのpNNx平均
        self.first_move = True # 最初の行動
        
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.vel = Twist()
        self.pub_pNNx = self.create_publisher(Int32, 'pNNx_plot', 10)
        self.pNNx = Int32() # pNNx
        self.pNNx.data = 0
        self.pub_pNNx_rest_ave = self.create_publisher(Int32, 'pNNx_rest_ave', 10)
        self.pNNx_rest_ave = Int32() # 何も無い時のpNNx平均
        self.pNNx_rest_ave.data = -1
        
        self.timer = self.create_timer(0.01, self.pNNx_callback)
    
    def pNNx_callback(self):
        sensor_val = self.ser.readline().decode(encoding='utf-8').strip()
        # RRI取得
        if sensor_val.startswith('Q'):
            print(f"センサーの値：{sensor_val}")
            print(f"安静時平均：{self.pNNx_rest_ave.data}, 直前平均：{self.pNNx_stimuli_ave}")
            self.RRI = int(sensor_val[1:])

            # データ数が足りてる時はpNNx計算
            if len(self.df) == 30:
                self.df[29] = self.RRI
                for i in range(29):
                    if self.df[i] - self.df[i+1] > self.x or self.df[i+1] - self.df[i] > self.x:
                        self.NNxCount += 1
                for i in range(29):
                    self.df[i] = self.df[i+1]

                # pNNxを計算して更新
                self.pNNx.data = int(self.NNxCount / 30 * 100)
                # pNNxの値に応じて動かす
                self.move()

            # pNNxに必要なデータ数が足りない時は貯める
            elif len(self.df) < 30:
                print(f"貯まったデータ数：{len(self.df)}")
                self.df.append(self.RRI)

            self.NNxCount = 0
            print(f"RRI: {self.RRI}, pNNx: {self.pNNx.data}\n")

    def move(self):
        # 安静時のpNNxデータを貯める
        if len(self.pNNx_rest) < 30:
            self.pNNx_rest.append(self.pNNx.data)
        # 安静時データが貯まっていてまだ平均を出していなければ出す
        elif self.pNNx_rest_ave.data == -1:
            self.pNNx_rest_ave.data = int(np.mean(self.pNNx_rest))
            self.pub_pNNx_rest_ave.publish(self.pNNx_rest_ave)
        # ロボ行動時のpNNxデータを貯める
        elif len(self.pNNx_stimuli) < 30:
            if self.first_move:
                self.vel.angular.z = 0.3
            self.pNNx_stimuli.append(self.pNNx.data)
        # ロボ行動時のpNNxデータが貯まっていれば平均を出す
        else:
            self.pNNx_stimuli_ave = np.mean(self.pNNx_stimuli)
            self.pNNx_stimuli.clear()

        # 安静時と直前のpNNxの平均を比較して動かす
        if self.pNNx_stimuli_ave > self.pNNx_rest_ave.data:
            # うれしい
            self.vel.angular.z = 0.5
        else:
            # 悲しい
            self.vel.angular.z = 0.1

        self.pub.publish(self.vel)
        self.pub_pNNx.publish(self.pNNx)

    def run(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        node.ser.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
