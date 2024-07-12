import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, String
from my_custom_message.msg import PulseData, BrainData
import math

class EmoStatusNode(Node):
    def __init__(self):
        super().__init__('emo_status_node')
        
        self.create_subscription(PulseData, 'pulse', self.pnnx_callback, 10)
        self.create_subscription(BrainData, 'brain_wave', self.beta_l_alpha_l_callback, 10)
        self.pnn50 = 0.0 # valence
        self.beta_l_alpha_l = 0 # arousal

        self.pub_emo_status = self.create_publisher(String, 'emo_status', 10)
        self.emo_status = String() # 感情状態
        self.emo_status.data = '' # 送信データ
        self.THRESHOLD = 0.236 # pnn50の平均指標
        self.THRESHOLD = 0.094 # pnn20の平均指標

        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.publish_emo_status)
    
    def pnnx_callback(self, msg):
        self.pnn50 = msg.pnn50
        self.get_logger().info(f'PNN50: {self.pnn50}')
    
    def beta_l_alpha_l_callback(self, msg):
        self.beta_l_alpha_l = msg.beta_l / msg.alpha_l
        self.get_logger().info(f'LOWBETA / LOWALPHA {self.beta_l_alpha_l}, {msg.beta_l}, {msg.alpha_l}')
    
    '''
    def cal_arousal(self, msg):
        tmp = msg.data.split(',')
        arousal = float(tmp[0]) - float(tmp[1]) # Attention - Meditation
        self.get_logger().info("arousal: " + str(arousal))
        return arousal
    '''

    def cal_valence(self):
        if (self.pnn50 < self.THRESHOLD):
            valence = (100 - ((self.pnn50 / self.THRESHOLD) * 100)) * (-1)
        else:
            valence = ((self.pnn50 - self.THRESHOLD) / (1 - self.THRESHOLD)) * 100
        self.get_logger().info("valence: " + str(valence))
        return valence

    def cal_angle(self, arousal, valence):
        angle = math.atan2(arousal, valence) * 180 / math.pi
        if (angle < 0):
            angle += 360
        angle = int(angle)
        self.get_logger().info("Angle: " + str(angle))
        return angle

    def cal_distance(self, arousal, valence):
        distance = math.sqrt(arousal*arousal + valence*valence)
        self.get_logger().info("Distance: " + str(distance))
        return distance

    def classify_emotion(self, angle):
        if angle <= 90:
            return 'Happy'
        elif angle <= 180:
            return 'Anger'
        elif angle <= 270:
            return 'Sadness'
        else:
            return 'Relax'

    def estimate_emotion(self):
        arousal = self.beta_l_alpha_l
        valence = self.cal_valence()

        angle = self.cal_angle(arousal, valence)
        emo_value = self.cal_distance(arousal, valence)
        emo_name = self.classify_emotion(angle)
        return [emo_name, emo_value]
    
    def publish_emo_status(self):
        sub_emo = self.estimate_emotion()

        self.emo_status.data = str(sub_emo[0]) + "," + str(sub_emo[1])
        self.pub_emo_status.publish(self.emo_status)
        self.get_logger().info(f'Emotion: {str(sub_emo[0])}, Value: {str(sub_emo[1])}\n\n')
 
    def run(self):
        while rclpy.ok():
            rclpy.spin(self)
            

def main(args=None):
    rclpy.init(args=args)
    talker = EmoStatusNode()
    try:
        talker.run()
    except KeyboardInterrupt:
        pass
    finally:
        talker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
