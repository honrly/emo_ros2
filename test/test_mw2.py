from NeuroPy import NeuroPy
from time import sleep

neuropy = NeuroPy("/dev/ttyS0", 9600)
print("b")
lowBeta_value = 0
lowAlpha_value = 0

def lowBeta_callback(value):
    """この関数は、NeuroPyがlowBeta新しい値を持つたびに呼び出される"""
    global lowBeta_value
    lowBeta_value = value
    print ("lowBeta値は: ", lowBeta_value)
    return None

def lowAlpha_callback(value):
    """この関数は、NeuroPyがlowAlphaの新しい値を持つたびに呼び出される"""
    global lowAlpha_value
    lowAlpha_value = value
    print ("lowAlpha値は: ", lowAlpha_value)
    return None

def calc_lowBeta_lowAlpha():
    global lowBeta_value, lowAlpha_value
    b_a = lowBeta_value / lowAlpha_value
    print("lowBeta/lowAlpha値は:", b_a)

# neuropy.start() の前に setCallBack を呼び出す
neuropy.setCallBack("lowAlpha", lowAlpha_callback)
neuropy.setCallBack("lowBeta", lowBeta_callback)

neuropy.start()
print("s")

try:
    while True:
        sleep(1) # CPUサイクルを食わないようにする
        if lowBeta_value != 0 and lowAlpha_value != 0:
            calc_lowBeta_lowAlpha()
finally:
    neuropy.stop()
