import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, String
import math

class EmoStatusNode(Node):
    def __init__(self):
        super().__init__('emo_status_node')
        
        self.create_subscription(Float32, 'pulse', self.pnnx_callback, 10)
        self.create_subscription(Int32, 'brain_wave', self.brain_wave_callback, 10)
        self.pnnx = 0.0 # valence
        self.att_med = 0 # arousal

        self.pub_emo_status = self.create_publisher(String, 'emo_status', 10)
        self.emo_status = String() # 感情状態
        self.emo_status.data = '' # 送信データ
        self.THRESHOLD = 0.236 # pnnxの平均指標

        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.publish_emo_status)
    
    def pnnx_callback(self, msg):
        self.pnnx = msg.data
    
    def brain_wave_callback(self, msg):
        self.att_med = msg.data
    
    '''
    def cal_arousal(self, msg):
        tmp = msg.data.split(',')
        arousal = float(tmp[0]) - float(tmp[1]) # Attention - Meditation
        self.get_logger().info("arousal: " + str(arousal))
        return arousal
    '''

    def cal_valence(self):
        if (self.pnnx < self.THRESHOLD):
            # 正規化、梶原さん
            valence = (100 - ((self.pnnx / self.THRESHOLD) * 100)) * (-1)
        else:
            valence = ((self.pnnx - self.THRESHOLD) / (1 - self.THRESHOLD)) * 100
        self.get_logger().info("valence: " + str(valence))
        return valence

    def cal_angle(self, arousal, valence):
        # 
        angle = math.atan2(arousal, valence) * 180 / math.pi
        if (angle < 0):
            angle += 360
        angle = int(angle)
        self.get_logger().info("Angle: " + str(angle))
        return angle

    def cal_distance(self, arousal, valence):
        # 無の感情基準、
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
        arousal = self.att_med
        valence = self.cal_valence()

        angle = self.cal_angle(arousal, valence)
        emo_value = self.cal_distance(arousal, valence)
        emo_name = self.classify_emotion(angle)
        return [emo_name, emo_value]
    
    def publish_emo_status(self):
        sub_emo = self.estimate_emotion()

        self.emo_status.data = str(sub_emo[0]) + "," + str(sub_emo[1])
        self.pub_emo_status.publish(self.emo_status)
        self.get_logger().info("Emotion: " + str(sub_emo[0]) + ", Value: " + str(sub_emo[1]))
 
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
