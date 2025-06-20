import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class PlotNode(Node):
    def __init__(self):
        super().__init__('plot_node')

        self.pnnx_rest_ave = -1
        self.fig, self.ax = plt.subplots()
        self.x = np.arange(30)
        self.y = np.zeros(30)
        self.y_rest = np.full(30, self.pnnx_rest_ave)  # pnnx_rest_ave の値で埋めた配列を作成
        self.line, = self.ax.plot(self.x, self.y)
        self.line_rest, = self.ax.plot(self.x, self.y_rest, 'r--')  # pnnx_rest_ave の線を赤色で追加

        self.create_subscription(Float32, 'pnnx', self.graph_callback, 10)
        self.create_subscription(Float32, 'pnnx_rest_ave', self.pnnx_rest_ave_callback, 10)
    
    def graph_callback(self, msg_pnnx):
        pnnx = msg_pnnx.data
        self.y = np.append(self.y[1:], pnnx) # 古いデータを捨てて新しいデータを追加

        self.line.set_ydata(self.y)
        self.ax.set_xlim((self.x.min(), self.x.max()))       
        self.ax.set_ylim(0, 100)

        plt.pause(0.01)

    def pnnx_rest_ave_callback(self, msg_pnnx_rest_ave):
        self.pnnx_rest_ave = msg_pnnx_rest_ave.data
        self.get_logger().info(f'Rest pnnx:{self.pnnx_rest_ave}')
        self.y_rest = np.full(30, self.pnnx_rest_ave)  # pnnx_rest_ave の値で埋めた配列を更新

        self.line_rest.set_ydata(self.y_rest)  # pnnx_rest_ave の線

def main(args=None):
    rclpy.init(args=args)
    listener = PlotNode()
    try:
        rclpy.spin(listener)
    except KeyboardInterrupt:
        pass
    finally:
        listener.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
