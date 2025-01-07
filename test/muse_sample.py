import time
import numpy as np
import pandas as pd
from brainflow.board_shim import BoardShim, BrainFlowInputParams, BoardIds
from brainflow.data_filter import DataFilter


EEG_LIST = ["RAW_TP9", "RAW_AF7", "RAW_AF8", "RAW_TP10"]
EEG_DF_COLUMN = ["package_num_channel", "RAW_TP9", "RAW_AF7", "RAW_AF8", "RAW_TP10", "other_channel", "time", "marker_channel"]
PPG_LIST = ["PPG_IR_16", "PPG_IR"]
PPG_DF_COLUMN = ["package_num_channel", "PPG_IR_16", "PPG_IR", "PPG_Red", "time", "marker_channel"]

params = BrainFlowInputParams()
params.mac_address = "00:55:DA:B9:7A:CE"

board = BoardShim(BoardIds.MUSE_S_BOARD, params)
board.prepare_session()

board.config_board("p50")
#board.config_board("p61")
board_id = BoardIds.MUSE_S_BOARD.value
print(BoardShim.get_board_presets(board_id))
'''
[2, 1, 0]
'''
print(BoardShim.get_board_descr(board_id, 0))
# print(BoardShim.get_board_descr(BoardIds.MUSE_S_BOARD.value, 1))
#print(BoardShim.get_board_descr(BoardIds.MUSE_S_BOARD.value, 2))
'''
{'eeg_channels': [1, 2, 3, 4], 'eeg_names': 'TP9,AF7,AF8,TP10', 'marker_channel': 7, 'name': 'MuseS', 'num_rows': 8, 'other_channels': [5], 'package_num_channel': 0, 'sampling_rate': 256, 'timestamp_channel': 6}
{'accel_channels': [1, 2, 3], 'gyro_channels': [4, 5, 6], 'marker_channel': 8, 'name': 'MuseSAux', 'num_rows': 9, 'package_num_channel': 0, 'sampling_rate': 52, 'timestamp_channel': 7}
{'marker_channel': 5, 'name': 'MuseSAnc', 'num_rows': 6, 'package_num_channel': 0, 'ppg_channels': [1, 2, 3], 'sampling_rate': 64, 'timestamp_channel': 4}

'''

board.start_stream()
time.sleep(10)
try:
    while True:
        data_eeg = board.get_board_data()
        
        #data_ppg = board.get_board_data(preset=2)

        print(data_eeg)
        print(len(data_eeg))
        print(data_eeg.shape)
        time.sleep(3)
        print('aaaa')
        
        #print(data_ppg)
        

except KeyboardInterrupt:
    pass
finally:
    board.stop_stream()
    board.release_session()