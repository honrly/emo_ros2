import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import serial
import numpy as np

MAX_LEN_DATA = 30
X_PNN = 50

class PnnxNode(Node):
    def __init__(self):
        super().__init__('pnnx_node')
        
        self.sensor = serial.Serial('/dev/ttyACM0', 115200)
        self.pub_pnnx = self.create_publisher(Int32, 'pnnx', 10)
        self.pnnx = Int32() # pnnx
        self.pnnx.data = 0 # 送信データ
        # pnnxの計算に使用
        self.rri = 0
        self.rri_arr = []
        self.xx_count = 0

        # 安静時のpnnx
        self.pub_pnnx_rest_ave = self.create_publisher(Int32, 'pnnx_rest_ave', 10)
        self.pnnx_rest_ave = Int32() # 何も無い時(安静時)のpnnx平均
        self.pnnx_rest_ave.data = -1 # 送信データ
        # 安静時のpnnxの計算に使用
        self.pnnx_rest = [] # 何も無い時のpnnxデータ(30個)
    
    def publish_pnnx(self):
        sensor_data = self.sensor.readline().decode(encoding='utf-8').strip()
        if sensor_data.startswith('Q'):
            self.get_logger.info(f'Sensor_data:{sensor_data}, pnnx:{self.pnnx.data}')
            # self.get_logger.info(f'安静時平均：{self.pnnx_rest_ave.data}, 直前平均：{self.pnnx_stimuli_ave}')
            self.rri = int(sensor_data[1:])
            self.xx_count += 1
            
            # データ数が足りてる時はpnnx計算
            if self.xx_count >= MAX_LEN_DATA + 2:
                self.rri_arr.pop(0)
                self.rri_arr.append(self.rri)
                count = 0
                for i in range(MAX_LEN_DATA):
                    if abs(self.rri_arr[i] - self.rri_arr[i+1]) > X_PNN:
                        count += 1

                # pnnxを計算して更新、publish
                self.pnnx.data = int((count / 30) * 100)
                self.pub_pnnx.publish(self.pnnx)
                # 安静時平均をpublish
                self.publish_pnnx_rest_ave(self.pnnx)
                self.get_logger.info(self.pnnx.data)

            # pnnxに必要なデータ数が足りない時は貯める
            elif self.xx_count < MAX_LEN_DATA + 2:
                self.rri_arr.append(self.rri)
                self.get_logger.info(f'貯まったデータ数：{len(self.rri_arr)}')
        
    def publish_pnnx_rest_ave(self, pnnx):
        # 安静時データを貯める
        if len(self.pnnx_rest) < MAX_LEN_DATA:
            self.pnnx_rest.append(pnnx.data)  # pnnx.data をリストに追加
        # 安静時データが貯まっていてまだ平均を出していなければ出す
        elif self.pnnx_rest_ave.data == -1:
            self.pnnx_rest_ave.data = int(np.mean(self.pnnx_rest))
            self.pub_pnnx_rest_ave.publish(self.pnnx_rest_ave)
    
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
