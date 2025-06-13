import openai
from google.cloud import texttospeech
import simpleaudio
import os
import speech_recognition as sr
from speech_recognition import AudioData
import time
import sounddevice as sd # alsa関連のwarnログが出るのを回避

from gtts import gTTS
from pygame import mixer

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, String

import cv2
import numpy as np
import os
import random
from random import randint
import threading


# APIキーの設定
openai.api_key = os.environ.get("OPENAI_API_KEY")
FLAG_EMOSUB = 0

# フレームバッファの設定（fbset で確認した値）
FB_WIDTH = 800
FB_HEIGHT = 480

def play(file):
    mixer.music.load(file)
    mixer.music.play()
    while mixer.music.get_busy():
        time.sleep(0.1)


class TalkEngNode(Node):
    def __init__(self):
        super().__init__('talk_eng')
        
        self.asr_text = self.create_publisher(String, 'asr_text', 10)
        self.robot_text = self.create_publisher(String, 'robot_text', 10)
        
        self.asr_text_data = String()
        self.asr_text_data.data = ""
        self.robot_text_data = String()
        self.robot_text_data.data = ""
        
        self.pub_face = self.create_publisher(String, 'face', 10)
        self.pub_motion = self.create_publisher(String, 'motion', 10)
        
        self.create_subscription(String, 'recog_emo', self.emo_callback, 10)
        self.emo = ''
        
        self.face = 'happy1'
        
        self.r = sr.Recognizer()
        self.mic = sr.Microphone()
        
        self.voice_list = ['Hello_WellcometoDolyLabo.mp3', 'Could_you_say_that_again.mp3', 'Could_you_say_that_again_justalittlebitlouder.mp3', 'Sorry_one_more_please.mp3']
        self.voice_dir = "/home/user/turtlebot3_ws/src/emotion_ros/Filler_EN/"
        self.files = os.listdir(self.voice_dir)
        
        self.filler_files = [f for f in self.files if f not in self.voice_list]
        mixer.init()

    
    def emo_callback(self, msg):
        global FLAG_EMOSUB
        self.emo = msg.data
        FLAG_EMOSUB = 1
        self.get_logger().info(f"Recog_emotion: {self.emo}")
    
    def face_motion(self):
        tmp = String()
        face_data = String()
        num = str(randint(1, 5))
        if self.emo == 'Happy':
            self.get_logger().info("\n#############\n### Happy ###\n#############")
            face_data.data = "happy" + num
            tmp.data = "LR,0.2,2.0,0.6,0"
            
        if self.emo == 'Relax':
            self.get_logger().info("\n#############\n### Relax ###\n#############")
            face_data.data = "relax" + num
            #tmp.data = "LR,0.0,2.0,0.3,0"
            tmp.data = "LR,0.2,2.0,0.2,0"
            
        if self.emo == 'Anger':
            self.get_logger().info("\n#############\n### Anger ###\n#############")
            face_data.data = "anger" + num
            tmp.data = "LR,0.4,2.0,0.6,0"
            
        if self.emo == 'Sadness':
            self.get_logger().info("\n###############\n### Sadness ###\n###############")
            face_data.data = "sadness" + num
            tmp.data = "LR,0.2,2.0,0.2,0"

        self.pub_motion.publish(tmp)
        self.pub_face.publish(face_data)
        # self.face = face_data.data
    
    def talk(self):        
        global FLAG_EMOSUB
        while FLAG_EMOSUB == 0:
            time.sleep(0.1)
        
        face_data = String()
        face_data.data = "happy1"
        self.pub_face.publish(face_data)
        
        hello = 'Hello_WellcometoDolyLabo.mp3'
        hello_file = os.path.join(self.voice_dir, hello)
        play(hello_file)
        
        while True:
            with self.mic as source:
                self.r.pause_threshold = 1
                #self.r.adjust_for_ambient_noise(source) #ノイズ除去
                self.r.adjust_for_ambient_noise(source, duration=1)  # 環境音を1秒間キャリブレーション
                #audio = self.r.listen(source)
                #audio = self.r.listen(source, timeout=5, phrase_time_limit=10)
                audio = self.r.listen(source, phrase_time_limit=10)

            start = time.perf_counter() #計測開始
            print ("Recognizing...")

            try:
                cmd = self.r.recognize_google(audio, language='en-US')
                print("You: {}".format(cmd))
                
                self.asr_text_data.data = cmd
                self.asr_text.publish(self.asr_text_data)

                # "ストップ"と発話することで、プログラムを止める
                #if self.r.recognize_google(audio, language='en-US') == "stop" :
                    #print("Quit the program")
                    #break

            except sr.UnknownValueError:
                self.get_logger().info("Couldn't recognize...")
                '''
                onemore_list = [f for f in self.voice_list if f != 'Hello_WellcometoDolyLabo.mp3']
                random_file = random.choice(onemore_list)
                self.get_logger().info(random_file)
                file_path = os.path.join(self.voice_dir, random_file)
                play(file_path)
                '''
                continue
            
            except sr.RequestError as e:
                self.get_logger().info("Could not request results from Google Speech Recognition service; {0}".format(e))
                '''
                onemore_list = [f for f in self.voice_list if f != 'Hello_WellcometoDolyLabo.mp3']
                random_file = random.choice(onemore_list)
                self.get_logger().info(random_file)
                file_path = os.path.join(self.voice_dir, random_file)
                play(file_path)
                '''
                continue
                
            except sr.WaitTimeoutError as e :
                self.get_logger().info("time out")
                continue

            random_file = random.choice(self.filler_files)
            self.get_logger().info(random_file)
            file_path = os.path.join(self.voice_dir, random_file)
            play(file_path)
            
            # GPTによる応答生成
            ##prompt = "以下の入力に対して優しい言葉で答えてください。\n入力:" + cmd + "\nこのとき、以下の条件1と条件2と条件3に従ってください。\n条件1:入力が挨拶のときは挨拶で返す、条件2:入力が質問のときは簡潔に回答する、条件3:入力が挨拶でも質問でもないときは入力に対して「ふんふん」のような相槌を打ってください。"
            #prompt = "Please respond to the following input in kind words. \nInput:" + cmd + "\nAt this time, please follow conditions 1, 2, and 3 below. \nCondition 1: If the input is a greeting, respond with a greeting. Condition 2: If the input is a question, reply briefly. Condition 3: If the input is neither a greeting nor a question, respond to the input with an appropriate response such as Hmmm."
            #prompt = "以下の入力と感情に対して優しい言葉で答えてください。\n入力:" + cmd + "\n感情:" + self.emo + "\nこのとき、以下の条件1と条件2と条件3に従ってください。\n条件1:入力が挨拶のときは感情に応じた挨拶で返す、条件2:入力が質問のときは簡潔に回答する、条件3:入力が挨拶でも質問でもないときは入力に対して「ふんふん」のような相槌を打ってください。"
            prompt = "Please respond to the following input and emotion in kind words. \nInput:" + cmd + "\nEmotion:" + self.emo + "\nAt this time, please follow conditions 1, 2, and 3 below. \nCondition 1: If the input is a greeting, respond with a greeting according to the emotion. Condition 2: If the input is a question, reply briefly. Condition 3: If the input is neither a greeting nor a question, respond to the input with an appropriate response such as Hmmm."
            #response = openai.ChatCompletion.create( 
            response = openai.chat.completions.create( 
                            #model = "gpt-3.5-turbo",
                            model = "gpt-4o-mini",
                            messages = [
                                {"role": "system", "content": "You are a helpful assistant."},
                                {"role": "user", "content": prompt}
                            ],
                            temperature=0
                        )

            # 応答の表示
            #text = response['choices'][0]['message']['content']
            text = response.choices[0].message.content
            self.get_logger().info("System: {}".format(text))
            
            self.robot_text_data.data = text
            self.robot_text.publish(self.robot_text_data)

            thread_b = threading.Thread(target=self.face_motion, daemon=True)
            thread_b.start()
            
            tts = gTTS(text, lang='en')
            tts.save('sample.mp3')
            
            play('sample.mp3')

    
    def start(self):
        a = threading.Thread(target=self.talk, daemon=True)
        a.start()

        
def main(args=None):
    rclpy.init(args=args)
    talker = TalkEngNode()
    try:
        talker.start()
        rclpy.spin(talker)
    except KeyboardInterrupt:
        pass
    finally:
        talker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
