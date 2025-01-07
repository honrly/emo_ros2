from bluepy.btle import Peripheral
import bluepy.btle as btle
import binascii
import matplotlib.pyplot as plt
import numpy as np
 
RRI_data = []
 
#ウェアラブルデバイスのアドレス（BLE）
ROHM_RAW = "E8:2C:41:93:A9:F2" #確認した心拍センサのアドレスを記入してください

# データを受信した時に呼ばれるclass
class MyDelegate(btle.DefaultDelegate):
    def __init__(self, params):
        btle.DefaultDelegate.__init__(self)
 
    def handleNotification(self, cHandle, data):
        global RRI_data
        c_data = binascii.b2a_hex(data)
        ##「c_data」が受信したデータ，デフォルトは16進数標記なので、10進数に戻す必要がある
        hr = int(c_data[2:4],16)
        v1 = int(c_data[4:6],16)
        v2 = int(c_data[6:8],16)
        rri1 = (v2<<8) + v1
        RRI_data = np.hstack([RRI_data, rri1])
        print(rri1)
        if len(c_data) > 8:
            v3 = int(c_data[8:10],16)
            v4 = int(c_data[10:12],16)
            rri2 = (v4<<8) + v3
            RRI_data = np.hstack([RRI_data, rri2])
            print(rri2)
            print(RRI_data)
 
# センサークラス
class SensorBLE(Peripheral):
    def __init__(self, addr):
        Peripheral.__init__(self, addr, addrType="random")
 
 
def main():
    # 初期設定
    medal = SensorBLE(ROHM_RAW)
    medal.setDelegate(MyDelegate(btle.DefaultDelegate))
 
    # ノーティフィケーションを有効にする
    #　Polar H10デバイスの場合は，0x0011に　0100を送れば有効になる
    medal.writeCharacteristic(0x0011, b"\x01\x00", True)
 
    # データを受信し続ける
    try:
        while True:
            if medal.waitForNotifications(1.0):##1秒間隔でデータを受け取る
                continue##continueにしておかないと、一定時間で接続が切れる？（要検証）
            print("wait...")
    except KeyboardInterrupt:
        pass
    finally:
        medal.disconnect()
if __name__ == "__main__":
    print("start")
    main()