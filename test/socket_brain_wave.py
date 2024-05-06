import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import socket
import pickle

class BrainWaveNode(Node):
    def __init__(self):
        super().__init__('brain_wave_node')

        self.receiver_ip = '192.168.65.29'
        self.receiver_port = 12345

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.receiver_ip, self.receiver_port))
        self.server_socket.listen(1)
        self.get_logger().info('Receiver:Connect Waiting...')
        self.connection, self.client_address = self.server_socket.accept()
        self.get_logger().info('Connected MindWave Server!')

        self.publisher = self.create_publisher(Float32, 'brain_wave', 10)
        self.lowb_lowa = Float32()
        self.lowb_lowa.data = 0.0

    def pub_lowb_lowa(self):
        while rclpy.ok():
            received_data = b''
            chunk = self.connection.recv(4096)
            received_data += chunk

            if not chunk:
                break

            deserialized_data = pickle.loads(received_data)
            self.lowb_lowa.data = deserialized_data['b_a']
            self.get_logger().info(f'Received lowb/lowa:{self.lowb_lowa.data}')

            self.publisher.publish(self.lowb_lowa)

def main(args=None):
    rclpy.init(args=args)
    talker = BrainWaveNode()
    try:
        talker.pub_lowb_lowa()
    finally:
        talker.connection.close()
        talker.server_socket.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
