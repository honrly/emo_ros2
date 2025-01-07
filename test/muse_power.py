import time
import numpy as np
from mne.time_frequency import psd_array_multitaper
from brainflow.board_shim import BoardShim, BrainFlowInputParams, BoardIds, BrainFlowPresets
from brainflow.exit_codes import BrainFlowExitCodes, BrainFlowError

def compute_band_power(data, sfreq, band):
    psds, freqs = psd_array_multitaper(data, sfreq=sfreq, fmin=band[0], fmax=band[1], verbose=False)
    return psds.mean(axis=-1)

# 周波数帯域の設定
LOW_ALPHA_BAND = (8, 9)  # 低アルファ波
LOW_BETA_BAND = (13, 17)  # 低ベータ波

EEG_CHANNELS = [2, 3]  # EEGチャンネル
SAMPLING_RATE = 256
RECONNECT_TIMEOUT = 2  # 再接続を試みるタイムアウト時間

af7_data = []
af8_data = []

def setup_muse():
    params = BrainFlowInputParams()
    params.mac_address = "00:55:DA:B9:7A:CE"

    board = BoardShim(BoardIds.MUSE_S_BOARD, params)
    max_retries = 10
    for attempt in range(max_retries):
        try:
            board.prepare_session()
            print("Board session prepared successfully.")
            break
        except Exception as e:
            print(f"Attempt {attempt + 1}/{max_retries} failed: {e}")
            if attempt < max_retries - 1:
                print(f"Retrying in 2 seconds...")
                time.sleep(2)
            else:
                print("Max retries reached. Failed to prepare board session.")

    board.config_board("p50")
    board_id = BoardIds.MUSE_S_BOARD.value
    print(BoardShim.get_board_descr(board_id, 0))

    board.start_stream()
    time.sleep(10)
    return board

board = setup_muse()

try:
    while True:
        # 生データを取得
        data_raw = board.get_board_data(preset=BrainFlowPresets.DEFAULT_PRESET)
        
        if data_raw.size > 0:
            last_data_time = time.time()  # データ更新時間を記録
            # チャンネルごとにデータを取得
            af7_data.extend(data_raw[EEG_CHANNELS[0]])
            af8_data.extend(data_raw[EEG_CHANNELS[1]])

            if len(af7_data) >= SAMPLING_RATE:
                start = time.time()

                # NumPy 配列に変換
                af7_array = np.array(af7_data)
                af8_array = np.array(af8_data)

                # 周波数解析
                low_alpha_power = compute_band_power(af7_array, SAMPLING_RATE, LOW_ALPHA_BAND)
                low_beta_power = compute_band_power(af7_array, SAMPLING_RATE, LOW_BETA_BAND)
                low_alpha_power2 = compute_band_power(af8_array, SAMPLING_RATE, LOW_ALPHA_BAND)
                low_beta_power2 = compute_band_power(af8_array, SAMPLING_RATE, LOW_BETA_BAND)

                # 結果を出力
                print(f"Low Alpha Power: {low_alpha_power}")
                print(f"Low Beta Power: {low_beta_power}")
                print(f"Low Alpha Power2: {low_alpha_power2}")
                print(f"Low Beta Power2: {low_beta_power2}")

                # バッファをリセット
                af7_data = []
                af8_data = []
                print(f"Processing Time: {time.time() - start:.2f} seconds")
        else:
            # 更新が一定時間ない場合に再接続を試みる
            if time.time() - last_data_time > RECONNECT_TIMEOUT:
                print("No data received for 2 seconds. Attempting to reconnect...")
                board.release_session()
                board = setup_muse()

except KeyboardInterrupt:
    pass
finally:
    board.stop_stream()
    board.release_session()
