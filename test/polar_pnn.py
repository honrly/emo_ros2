import math
from concurrent.futures import ThreadPoolExecutor
from bluepy.btle import Peripheral
import bluepy.btle as btle
import binascii
import numpy as np
import time
 
# 定数
IBI_SIZE = 31

# グローバル変数
ibi_list = [0] * IBI_SIZE
ibi_count = 0
msg = {}

RRI_data = []

#ウェアラブルデバイスのアドレス（BLE）
ROHM_RAW = "C0:FB:69:89:1A:DA" #確認した心拍センサのアドレスを記入してください

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


def calc_hrv():
    global ibi_list
    mean = sum(ibi_list) / IBI_SIZE
    calc_sdnn(mean)

def calc_sdnn(mean):
    global ibi_list, msg
    variance_sum = sum((ibi - mean) ** 2 for ibi in ibi_list)
    sdnn = math.sqrt(variance_sum / IBI_SIZE)
    msg['sdnn'] = sdnn

    calc_cvnn(mean, sdnn)
    calc_rmssd()
    calc_pnn()

def calc_cvnn(mean, sdnn):
    global msg
    cvnn = sdnn / mean
    msg['cvnn'] = cvnn

def calc_rmssd():
    global ibi_list, msg
    diff_squares_sum = sum((ibi_list[i] - ibi_list[i - 1]) ** 2 for i in range(1, IBI_SIZE))
    rmssd = math.sqrt(diff_squares_sum / IBI_SIZE)
    msg['rmssd'] = rmssd

def calc_pnn():
    global ibi_list, msg
    nn10 = nn20 = nn30 = nn40 = nn50 = 0

    for i in range(1, IBI_SIZE):
        nnx = abs(ibi_list[i] - ibi_list[i - 1])
        if nnx > 50:
            nn50 += 1
        if nnx > 40:
            nn40 += 1
        if nnx > 30:
            nn30 += 1
        if nnx > 20:
            nn20 += 1
        if nnx > 10:
            nn10 += 1

    msg['pnn50'] = nn50 / (IBI_SIZE - 1)
    msg['pnn40'] = nn40 / (IBI_SIZE - 1)
    msg['pnn30'] = nn30 / (IBI_SIZE - 1)
    msg['pnn20'] = nn20 / (IBI_SIZE - 1)
    msg['pnn10'] = nn10 / (IBI_SIZE - 1)

def main():
    # 初期設定
    global RRI_data, ibi_count
    
    while True:  # 再接続処理を繰り返すためのループ
        try:
            medal = SensorBLE(ROHM_RAW)
            medal.setDelegate(MyDelegate(btle.DefaultDelegate))
            
            # ノーティフィケーションを有効にする
            medal.writeCharacteristic(0x0011, b"\x01\x00", True)
            
            # データを受信し続ける
            while True:
                if medal.waitForNotifications(1.0):  # 1秒間隔でデータを受け取る
                    start = time.time()
                    if len(RRI_data) != 0:
                        for ibi in RRI_data:
                            msg['ibi'] = ibi  # 最後の値をメッセージに記録

                            if ibi_count >= IBI_SIZE:
                                ibi_list.pop(0)  # 最も古い値を削除
                                ibi_list.append(ibi)  # 新しい値を追加
                            elif ibi_count >= 0:
                                ibi_list[ibi_count] = ibi  # 配列に値を追加
                                ibi_count += 1
                            else:
                                raise Exception("IBI count error")

                        RRI_data = []
                        calc_hrv()
                        print(msg)  # 結果を出力

                    continue  # 一定時間で接続が切れないように継続

        except btle.BTLEDisconnectError as e:
            print(f"再接続を試みます: {e}")
            time.sleep(1)  # 再接続前に待機
            continue  # 再接続ループを再開

        except ValueError as e:
            print(f"データエラー: {e}")
            # medalをクリーンアップして再接続
            try:
                medal.writeCharacteristic(0x0011, b"\x00\x00", True)
                medal.disconnect()
            except Exception as cleanup_error:
                print(f"クリーンアップ中にエラー: {cleanup_error}")
                continue
            time.sleep(1)

        except KeyboardInterrupt:
            print("終了します。")
            break

        finally:
            try:
                if 'medal' in locals() or 'medal' in globals():
                    medal.writeCharacteristic(0x0011, b"\x00\x00", True)
                    medal.disconnect()
            except Exception:
                pass
        
if __name__ == "__main__":
    print("start")
    main()