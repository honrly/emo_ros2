import rclpy
import serial
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import numpy as np

MAX_LEN_DATA = 30

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        
        self.first_move = True # 最初のロボの行動
        
        self.pNNx_rest = [] # 何も無い時(安静時)のpNNxデータ(30個)
        self.pNNx_stimuli = [] # ロボが行動したときのpNNxデータ(30個)
        
        self.pNNx_stimuli_ave = -1 # ロボが行動したときのpNNx平均
        
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.vel = Twist()
        self.pub_pNNx = self.create_publisher(Int32, 'pNNx_plot', 10)
        self.pNNx = Int32() # pNNx
        self.pNNx.data = 0 # 格納、送信データ
        self.pub_pNNx_rest_ave = self.create_publisher(Int32, 'pNNx_rest_ave', 10)
        self.pNNx_rest_ave = Int32() # 何も無い時のpNNx平均
        self.pNNx_rest_ave.data = -1 # 格納、送信データ
        
        self.timer = self.create_timer(0.01, self.pNNx_callback)
    
    def serial(self):
        SENSOR = serial.Serial('/dev/ttyACM0', 115200)
        X_PNN = 50
        rri = 0
        rri_arr = [] # 計算対象データ(30個想定)
        xx_count = 0
        pNNx = 0
        
        while True:
            try:
                sensor_data = SENSOR.readline().decode(encoding='utf-8').strip()
                if sensor_data.startswith('Q'):
                    print(f"センサーの値：{sensor_data}")
                    print(f"安静時平均：{self.pNNx_rest_ave.data}, 直前平均：{self.pNNx_stimuli_ave}")
                    
                    rri = int(sensor_data[1:])
                    xx_count += 1

                    # データ数が足りてる時はpNNx計算
                    if len(self.df) == 30:
                        rri_arr.pop(0)
                        rri_arr.append(rri)
                        count = 0
                        for i in range(MAX_LEN_DATA):
                            if abs(rri_arr[i] - rri_arr[i+1]) > X_PNN:
                                count += 1

                        # pNNxを計算して更新
                        pNNx = count / 30
                        self.pNNx.data = pNNx
                        # pNNxの値に応じて動かす
                        self.move()

                    # pNNxに必要なデータ数が足りない時は貯める
                    elif xx_count < MAX_LEN_DATA + 2:
                        rri_arr.append(rri)
                        print(f"貯まったデータ数：{len(rri_arr)}")
                        
                    print(f"RRI: {self.RRI}, pNNx: {self.pNNx.data}\n")
            except Exception as e:
                print("Error reading serial data:", str(e))
                break

    def move(self):
        # 安静時のpNNxデータを貯める
        if len(self.pNNx_rest) < MAX_LEN_DATA:
            self.pNNx_rest.append(self.pNNx.data)
        # 安静時データが貯まっていてまだ平均を出していなければ出す
        elif self.pNNx_rest_ave.data == -1:
            self.pNNx_rest_ave.data = int(np.mean(self.pNNx_rest))
            self.pub_pNNx_rest_ave.publish(self.pNNx_rest_ave)
        # ロボ行動時のpNNxデータを貯める
        elif len(self.pNNx_stimuli) < MAX_LEN_DATA:
            if self.first_move:
                self.vel.angular.z = 0.3
                self.first_move = False
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

def main(args=None):
    rclpy.init(args=args)
    talker = SerialNode()

    try:
        rclpy.spin(talker)
    except KeyboardInterrupt:
        pass
    finally:
        talker.destroy_node()
        talker.ser.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
