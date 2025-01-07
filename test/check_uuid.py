from bluepy import btle

BLE_ADDRESS="E8:2C:41:93:A9:F2"

#https://qiita.com/gebo/items/7978fe95c29c6bf7a0f4
#https://github.com/polarofficial/polar-ble-sdk/blob/187098ff594e294a72759c37372b1970611b1a83/technical_documentation/Polar_Measurement_Data_Specification.pdf

peripheral = btle.Peripheral()
peripheral.connect(BLE_ADDRESS, addrType=btle.ADDR_TYPE_RANDOM)
print('a')
for service in peripheral.getServices():
    print(f'Service UUID：{service.uuid}')
    for characteristic in service.getCharacteristics():
        print(f'- Characteristic UUID：{characteristic.uuid} , Handle：{hex(characteristic.getHandle())} , Property：{characteristic.propertiesToString()}')

'''
Service UUID：00001800-0000-1000-8000-00805f9b34fb
- Characteristic UUID：00002a00-0000-1000-8000-00805f9b34fb , Handle：0x3 , Property：READ 
- Characteristic UUID：00002a01-0000-1000-8000-00805f9b34fb , Handle：0x5 , Property：READ 
- Characteristic UUID：00002a04-0000-1000-8000-00805f9b34fb , Handle：0x7 , Property：READ 
- Characteristic UUID：00002aa6-0000-1000-8000-00805f9b34fb , Handle：0x9 , Property：READ 
Service UUID：00001801-0000-1000-8000-00805f9b34fb
- Characteristic UUID：00002a05-0000-1000-8000-00805f9b34fb , Handle：0xc , Property：INDICATE 
Service UUID：0000180d-0000-1000-8000-00805f9b34fb
- Characteristic UUID：00002a37-0000-1000-8000-00805f9b34fb , Handle：0x10 , Property：NOTIFY 
- Characteristic UUID：00002a38-0000-1000-8000-00805f9b34fb , Handle：0x13 , Property：READ 
Service UUID：0000180a-0000-1000-8000-00805f9b34fb
- Characteristic UUID：00002a29-0000-1000-8000-00805f9b34fb , Handle：0x16 , Property：READ 
- Characteristic UUID：00002a24-0000-1000-8000-00805f9b34fb , Handle：0x18 , Property：READ 
- Characteristic UUID：00002a25-0000-1000-8000-00805f9b34fb , Handle：0x1a , Property：READ 
- Characteristic UUID：00002a27-0000-1000-8000-00805f9b34fb , Handle：0x1c , Property：READ 
- Characteristic UUID：00002a26-0000-1000-8000-00805f9b34fb , Handle：0x1e , Property：READ 
- Characteristic UUID：00002a28-0000-1000-8000-00805f9b34fb , Handle：0x20 , Property：READ 
- Characteristic UUID：00002a23-0000-1000-8000-00805f9b34fb , Handle：0x22 , Property：READ 
Service UUID：0000180f-0000-1000-8000-00805f9b34fb
- Characteristic UUID：00002a19-0000-1000-8000-00805f9b34fb , Handle：0x25 , Property：READ NOTIFY 
Service UUID：6217ff4b-fb31-1140-ad5a-a45545d7ecf3
- Characteristic UUID：6217ff4c-c8ec-b1fb-1380-3ad986708e2d , Handle：0x29 , Property：READ 
- Characteristic UUID：6217ff4d-91bb-91d0-7e2a-7cd3bda8a1f3 , Handle：0x2b , Property：WRITE NO RESPONSE INDICATE 
Service UUID：fb005c80-02e7-f387-1cad-8acd2d8df0c8
- Characteristic UUID：fb005c81-02e7-f387-1cad-8acd2d8df0c8 , Handle：0x2f , Property：READ WRITE INDICATE 
- Characteristic UUID：fb005c82-02e7-f387-1cad-8acd2d8df0c8 , Handle：0x32 , Property：NOTIFY 
Service UUID：0000feee-0000-1000-8000-00805f9b34fb
- Characteristic UUID：fb005c51-02e7-f387-1cad-8acd2d8df0c8 , Handle：0x36 , Property：WRITE NO RESPONSE WRITE NOTIFY 
- Characteristic UUID：fb005c52-02e7-f387-1cad-8acd2d8df0c8 , Handle：0x39 , Property：NOTIFY 
- Characteristic UUID：fb005c53-02e7-f387-1cad-8acd2d8df0c8 , Handle：0x3c , Property：WRITE NO RESPONSE WRITE 

'''

peripheral.disconnect()
print("Disconnected from the BLE device.")
