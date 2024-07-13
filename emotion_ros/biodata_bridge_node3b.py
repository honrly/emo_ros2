#!/usr/bin/env python3
# coding: utf-8

# ソケット通信(クライアント側)
import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.parameter import Parameter

class BiodataBridge(Node):
  def __init__(self):
    super().__init__('biodata_bridge_node3b')
    
    ip1 = 'localhost'
    #self.declare_parameter('server', 'localhost')
    #ip1 = str(self.get_parameter('server'))
    port1 = 8765

    server1 = (ip1, port1)
    socket1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    socket1.connect(server1)

    pub_hrveeg_data = self.create_publisher(String, 'hrveeg_data', 10)

    while rclpy.ok():
        # サーバからデータを受信
        recvline = socket1.recv(4096).decode()
        #self.get_logger().info(recvline)
        #print('サーバーからのデータ: ' + str(recvline))
        self.get_logger().info("サーバーからのデータ: " + str(recvline))

        # [Attention，Meditation，BPM，IBI，pNN50]
        vals = recvline.split(',')
        #print('リスト化: ' + str(vals))
        self.get_logger().info("リスト化: " + str(vals))

        _Attention = int(vals[0])   # uint8
        _Meditation = int(vals[1])  # uint8
        _BPM = int(vals[2])         # uint8
        _IBI = int(vals[3])         # uint16
        _pNN50 = float(vals[4])     # float32

        #self.get_logger().info("Att.: %d, Med.: %d, BPM: %d, IBI: %d, pNN50: %f", _Attention, _Meditation, _BPM, _IBI, _pNN50)
        self.get_logger().info("Att.: " + str(_Attention) + ", Med.: " + str(_Meditation) + ", BPM: " + str(_BPM) + ", IBI: " + str(_IBI) + ", pNN50: " + str(_pNN50))
        #print('Attention:  ' + str(_Attention))
        #print('Meditation: ' + str(_Meditation))
        #print('BPM:        ' + str(_BPM))
        #print('IBI:        ' + str(_IBI))
        #print('pNN50:      ' + str(_pNN50))

        hrveeg_data = String()
        hrveeg_data.data = str(_Attention) + "," + str(_Meditation) + "," + str(_BPM) + "," + str(_IBI) + "," + str(_pNN50) 
        pub_hrveeg_data.publish(hrveeg_data)
        
    socket1.close()
    print('クライアント側終了です')

def main(args=None):
  try:
    rclpy.init(args=args)
    talker = BiodataBridge()
    rclpy.spin(talker)
  except KeyboardInterrupt:
    pass
  finally:
    talker.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()
