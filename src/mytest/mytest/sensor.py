import rclpy
import serial
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import numpy as np
import concurrent.futures

# PNNX = 0
MAX_LEN_DATA = 30

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        
        self.pub_pNNx = self.create_publisher(Int32, 'serial_pNNx', 10)
        self.pNNx = Int32() # pNNx
        self.pNNx.data = 0 # 格納、送信データ
    
    def serial_reader(self):
        SENSOR = serial.Serial('/dev/ttyACM0', 115200)
        # global PNNX
        X_PNN = 50
        rri = 0
        rri_arr = []
        xx_count = 0
        while True:
            sensor_data = SENSOR.readline().decode(encoding='utf-8').strip()
            if sensor_data.startswith('Q'):
                print(f"センサーの値：{sensor_data}")
                # print(f"安静時平均：{self.pNNx_rest_ave.data}, 直前平均：{self.pNNx_stimuli_ave}")
                rri = int(sensor_data[1:])
                xx_count += 1
                
                # データ数が足りてる時はpNNx計算
                if xx_count >= MAX_LEN_DATA + 2:
                    rri_arr.pop(0)
                    rri_arr.append(rri)
                    count = 0
                    for i in range(MAX_LEN_DATA):
                        if abs(rri_arr[i] - rri_arr[i+1]) > X_PNN:
                            count += 1

                    # pNNxを計算して更新
                    self.pNNx = count / 30
                    self.pub_pNNx.publish(self.pNNx)

                # pNNxに必要なデータ数が足りない時は貯める
                elif xx_count < MAX_LEN_DATA + 2:
                    rri_arr.append(rri)
                    print(f"貯まったデータ数：{len(rri_arr)}")
        
    def run(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    talker = SerialNode()
    try:
        talker.run()
    except KeyboardInterrupt:
        pass
    finally:
        talker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
