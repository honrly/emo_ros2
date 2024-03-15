import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Nodeクラスを継承
class SerialListener(Node):
    def __init__(self):
        # 引数node_nameにlistenerを渡す。
        super().__init__('serial_listener')

        self.ser = serial.Serial('/dev/ttyACM0', 9600)
        self.RRI = 0
        self.NNxCount = 0 # RRIの差が大きい瞬間数
        self.pNNx = 0 # pNNx
        self.x = 50 # pNNx
        self.df = [] # 計算対象データ(30個想定)
        self.pNNx_rest = [] # 何も無い時(安静時)のpNNxデータ(30個)
        self.pNNx_stimuli = [] # ロボが行動したときのpNNxデータ(30個)
        self.pNNx_rest_ave = 0 # 何も無い時のpNNx平均
        self.pNNx_stimuli_ave = 0 # ロボが行動したときのpNNx平均
        
        # 引数msg_type、topic_name、callbackに渡してSubscriptionを作成
        self.create_subscription(String, 'serial_sub', self.pNNx_callback)

    # 処理用callback関数
    def pNNx_callback(self):
        sensor_val = self.ser.readline().decode().strip()
        print(sensor_val)
        # RRI取得
        if sensor_val and sensor_val[0] == 'Q':
            print("a")
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
                self.pNNx = self.NNxCount / 30 * 100

                # 安静時のpNNxデータを貯める
                if len(self.pNNx_rest) < 30:
                    self.pNNx_rest.append(self.pNNx)
                # 安静時データが貯まっていてまだ平均を出していなければ出す
                elif self.pNNx_rest_ave == 0:
                    self.pNNx_rest_ave = np.mean(self.pNNx_rest)
                # ロボ行動時のpNNxデータを貯める
                elif self.pNNx_stimuli < 30:
                    self.pNNx_stimuli.append(self.pNNx)
                # 貯まっていて平均を出していなければ出す
                elif self.pNNx_stimuli_ave == 0:
                    self.pNNx_stimuli_ave = np.mean(self.pNNx_stimuli)
                elif self.pNNx_stimuli_ave > self.pNNx_rest_ave:
                    # うれしい
                    print("a")
                else:
                    # 謝る
                    print("b")

            # pNNxに必要なデータ数が足りない時は貯める
            elif len(self.df) < 30:
                print(f"貯まったデータ数：{len(self.df)}")
                self.df.append(self.RRI)

            print("RRI: ", self.RRI, ", pNNx: ", self.pNNx)

            # pNNxの値に応じて動かす
            # self.move()

            self.NNxCount = 0


def main():
    # RCLの初期化を実行
    rclpy.init()

    # Listenerクラスのコンスタンス化
    node = SerialListener()

    # ループに入りnode内の処理を実行させる
    rclpy.spin(node)

    # nodeを破壊
    node.destroy_node()

    # RCLをシャットダウン
    rclpy.shutdown()

if __name__ == "__main__":
    main()
