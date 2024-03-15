import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Nodeクラスを継承
class PlotNode(Node):
    def __init__(self):
        super().__init__('plot_node')

        self.pNNx_rest_ave = -1
        self.fig, self.ax = plt.subplots()
        self.x = np.arange(30)
        self.y = np.zeros(30)
        self.line, = self.ax.plot(self.x, self.y)

        self.create_subscription(Int32, 'pNNx_rest_ave', self.pNNx_rest_ave_callback, 10)
        self.create_subscription(Int32, 'pNNx_plot', self.graph_callback, 10)
    
    def graph_callback(self, msg_pNNx):
        pNNx = msg_pNNx.data
        self.y = np.append(self.y[1:], pNNx) # 古いデータを捨てて新しいデータを追加

        self.line.set_ydata(self.y)
        self.ax.set_xlim((self.x.min(), self.x.max()))

        y_min = np.min(self.y)
        y_max = np.max(self.y)
        if self.pNNx_rest_ave == -1:
            self.ax.set_ylim(0, 100)
        else:
            center = abs(pNNx - self.pNNx_rest_ave)
            self.ax.set_ylim(max(0, y_min - 10), 
                             y_max + 10)
            
            # self.ax.set_ylim(0, 100)

        plt.pause(0.01)

    def pNNx_rest_ave_callback(self, msg_pNNx_rest_ave):
        self.pNNx_rest_ave = msg_pNNx_rest_ave.data
        
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


if __name__ == "__main__":
    main()
