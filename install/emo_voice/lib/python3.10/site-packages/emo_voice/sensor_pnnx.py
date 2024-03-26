import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import serial
import numpy as np

SENSOR = serial.Serial('/dev/ttyACM0', 115200)
MAX_LEN_DATA = 30
X_PNN = 50

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        
        self.pub_pnnx = self.create_publisher(Int32, 'sensor_pnnx', 10)
        self.pnnx = Int32() # pnnx
        self.pnnx.data = 0 # 送信データ

        self.rri = 0
        self.rri_arr = []
        self.xx_count = 0
    
    def calc_pnnx(self):
        sensor_data = SENSOR.readline().decode(encoding='utf-8').strip()
        if sensor_data.startswith('Q'):
            print(f"センサーの値：{sensor_data}, pnnx:{self.pnnx.data}")
            # print(f"安静時平均：{self.pnnx_rest_ave.data}, 直前平均：{self.pnnx_stimuli_ave}")
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
                print(self.pnnx.data)

            # pnnxに必要なデータ数が足りない時は貯める
            elif self.xx_count < MAX_LEN_DATA + 2:
                self.rri_arr.append(self.rri)
                print(f"貯まったデータ数：{len(self.rri_arr)}")
        
    def run(self):
        while rclpy.ok():
            self.calc_pnnx()

def main(args=None):
    rclpy.init(args=args)
    talker = SensorNode()
    try:
        talker.run()
    except KeyboardInterrupt:
        pass
    finally:
        talker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
