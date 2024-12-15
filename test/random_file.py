import os
import random

filler_dir = f"/home/user/turtlebot3_ws/src/emotion_ros/Filler_JP/"

# ディレクトリ内のファイル一覧を取得
try:
    files = os.listdir(filler_dir)
    # ファイルが存在する場合にランダムで1つ選択
    if files:
        random_file = random.choice(files)
        print(f"ランダムに選ばれたファイル: {random_file}")
    else:
        print("指定したディレクトリにはファイルがありません。")
except FileNotFoundError:
    print(f"指定したディレクトリが見つかりません: {filler_dir}")
