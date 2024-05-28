import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib import patches
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.animation import FuncAnimation

def plot_emotion_map(ax):
    # 正方形
    ax.set_aspect('equal')
    ax.set_ylim([-4,4])
    ax.set_xlim([-4,4])
    ax.set_xticks([]) # 数字目盛りをなくす
    ax.set_yticks([])

    # 十字とバツ
    ax.axhline(0, color='k', lw=0.5)
    ax.axline((0,0), slope=1.0, color='k', lw=0.5)
    ax.axvline(0, color='k', lw=0.5)
    ax.axline((0,0), slope=-1.0, color='k', lw=0.5)

    # 同心円
    for radius in range(1, 5):
        circle = patches.Circle(xy=(0,0), radius=radius, fill=False, lw=0.5)
        ax.add_patch(circle)

    # 感情のテキスト
    labels = ['Excited', 'Happy', 'Relax', 'Sleepy', 'Tired', 'Gloomy', 'Angry', 'Alarmed']
    positions = [
        (2, 4.5), (5.3, 2),
        (5.3, -2), (2, -4.5),
        (-2, -4.5), (-5.3, -2),
        (-5.3, 2), (-2, 4.5)
    ]
    
    for (x, y), label in zip(positions, labels):
        ax.text(x, y, label, ha='center', va='center')

    # 例
    theta = np.deg2rad(30)
    wedge = plt.Polygon([[0, 0], [np.cos(theta), np.sin(theta)], [1, 0]], closed=True, color='yellow', alpha=0.5)
    ax.add_artist(wedge)

    # 線
    ax.plot([0, np.cos(theta)], [0, np.sin(theta)], 'r-', lw=2)
    ax.plot([0, 1], [0, 0], 'g-', lw=2)

    # 点
    ax.plot(np.cos(theta), np.sin(theta), 'ko')

def plot_bio_table(ax):
    ax.axis('off') 
    ax.axis('tight')

    data = {
        'Label': ['Timestamp', 'BPM', 'IBI', 'pNN50', 'Attention', 'Meditation', 'poorsignal', 'pNN20'],
        'Value': ['12:12:13', 34, 700, 0.600, 65, 62, 0, 0.800]
    }
    df = pd.DataFrame(data)

    tb = ax.table(cellText=df.values,
                  colLabels=None,
                  cellLoc='center',
                  colWidths=[0.3, 0.2],
                  bbox=[0, 0, 1, 1])

    for i in range(len(df)):
        for j in range(2): # 列
            cell = tb[i, j]
            cell.set_edgecolor('#555555')
            cell.set_text_props(fontsize=20, fontweight='bold')
            if i % 2 == 1:
                cell.set_facecolor('#dcdcdc')

fig = plt.figure(figsize=(12.0, 7.0))
gs = gridspec.GridSpec(13, 23)

# グリッドの設定
ax_line = plt.subplot(gs[0:13, 0:14])
ax_emo_map_pnn20 = plt.subplot(gs[0:6, 15:18])
ax_emo_map_pnn50 = plt.subplot(gs[0:6, 20:23])
ax_bio_table = plt.subplot(gs[7:13, 15:23])

# サブプロットの周囲に余白を設けるための調整
plt.subplots_adjust(left=0.05, right=0.95, bottom=0.05, top=0.95, wspace=0.3, hspace=0.3)

# 感情マップのプロット
plot_emotion_map(ax_emo_map_pnn20)
plot_emotion_map(ax_emo_map_pnn50)
plot_bio_table(ax_bio_table)

plt.show()
