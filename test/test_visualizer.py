import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib import patches
import pandas as pd
import datetime
import math

def range_transform(input, before_min, before_max, after_min, after_max):
    return after_min + (after_max - after_min) * ((input - before_min) / (before_max - before_min))

def plot_emotion_map(ax):
    pnnx_input, brain_input = 0.7, 0.2
    
    pnnx_min, pnnx_max = 0.0, 1.0
    brain_min, brain_max = 0.0, 2.0
    graph_min, graph_max = -4.0, 4.0
    pnnx_center = 0.236
    brain_center = 1.0
    
    ax.set_aspect('equal') # 正方形比にする
    ax.set_ylim([-4, 4])
    ax.set_xlim([-4, 4])
    ax.set_xticks([]) # 目盛りをなくす
    ax.set_yticks([])

    ax.axhline(0, color='k', lw=0.5) # 横線
    ax.axline((0, 0), slope=1.0, color='k', lw=0.5) # 斜め線
    ax.axvline(0, color='k', lw=0.5) # 縦線
    ax.axline((0, 0), slope=-1.0, color='k', lw=0.5)

    # 同心円
    for radius in range(1, 5):
        circle = patches.Circle(xy=(0, 0), radius=radius, fill=False, lw=0.5)
        ax.add_patch(circle)

    labels = ['Excited', 'Happy', 'Relaxed', 'Sleepy', 'Tired', 'Gloomy', 'Angry', 'Alarmed']
    positions = [
        (2, 4.5), (5.3, 2),
        (5.3, -2), (2, -4.5),
        (-2, -4.5), (-5.3, -2),
        (-5.3, 2), (-2, 4.5)
    ]
    
    # 文字の配置
    for (x, y), label in zip(positions, labels):
        ax.text(x, y, label, ha='center', va='center', size=15)

    # 点    
    mapped_pnnx, mapped_brain = 0.0, 0.0
    if (pnnx_input < pnnx_center):
        mapped_pnnx = range_transform(pnnx_input, pnnx_min, pnnx_center, graph_min, 0)
    else:
        mapped_pnnx = range_transform(pnnx_input, pnnx_center, pnnx_max, 0, graph_max)

    if (brain_input < brain_center):
        mapped_brain = range_transform(brain_input, brain_min, brain_center, graph_min, 0)
    else:
        mapped_brain = range_transform(brain_input, brain_center, brain_max, 0, graph_max)

    # ax.plot(np.cos(theta), np.sin(theta), 'ko')
    ax.plot(mapped_pnnx, mapped_brain, 'ko')
    angle = math.degrees(math.atan2(mapped_brain, mapped_pnnx))
    print(angle)
    
    emotion_color = [
        ["#d1e8ae", "#bdea75", "#a8ed3b", "#93ef00"], # 0~44度
        ["#e6e8ae", "#e6ea75", "#e7ed3b", "#e7ef00"], # 45~89度
        ["#e8c4ae", "#eaa275", "#ed823b", "#ef6300"], # 90~134度
        ["#e8aeae", "#ea7575", "#ed3b3b", "#ef0000"],  # 135~179度
        ["#ccaee8", "#b375ea", "#9a3bed", "#8300ef"], # 180~224度
        ["#b1b4ed", "#757dea", "#3a48e8", "#000bef"], # 225~269度
        ["#aee8d5", "#75eac5", "#3bedb7", "#00efab"], # 270~314度
        ["#aee8af", "#75ea77", "#3bed3e", "#00ef07"], # 315~359度   
    ]

    # 座標から角度
    angle = math.degrees(math.atan2(mapped_brain, mapped_pnnx)) % 360

    # 半径
    r = math.sqrt(mapped_pnnx**2 + mapped_brain**2)

    # 塗る範囲
    if r > 3:
        ranges = 4
    elif 2 < r <= 3:
        ranges = 3
    elif 1 < r <= 2:
        ranges = 2
    else:
        ranges = 1
    
    # 角度から色
    if 0 <= angle < 45:
        row_index = 0
        angle_start = 0
        angle_end = 45
    elif 45 <= angle < 90:
        row_index = 1
        angle_start = 45
        angle_end = 90
    elif 90 <= angle < 135:
        row_index = 2
        angle_start = 90
        angle_end = 135
    elif 135 <= angle < 180:
        row_index = 3
        angle_start = 135
        angle_end = 180
    elif 180 <= angle < 225:
        row_index = 4
        angle_start = 180
        angle_end = 225
    elif 225 <= angle < 270:
        row_index = 5
        angle_start = 225
        angle_end = 270
    elif 270 <= angle < 315:
        row_index = 6
        angle_start = 270
        angle_end = 315
    else:
        row_index = 7
        angle_start = 315
        angle_end = 360

    # 色塗り
    for i in range(ranges):
        color = emotion_color[row_index][i]
        wedge = patches.Wedge(center=(0, 0), r=i+1, theta1=angle_start, theta2=angle_end, width=1, color=color)
        ax.add_patch(wedge)

    # グラフの外枠と同心円の間
    if r > 3:
        outer_wedge = patches.Wedge(center=(0, 0), r=6, theta1=angle_start, theta2=angle_end, width=2, color=emotion_color[row_index][3])
        ax.add_patch(outer_wedge)
        
        
def plot_bio_table(ax):
    ax.axis('off')
    ax.axis('tight')
    
    t_delta = datetime.timedelta(hours=9)
    JST = datetime.timezone(t_delta, 'JST')
    now = datetime.datetime.now(JST)
    d = now.strftime('%H:%M:%S')

    data = {
        'Label': ['Timestamp', 'BPM', 'IBI', 'pNN20', 'pNN50', 'Attention', 'Meditation', 'poorsignal'],
        'Character': [':']*8,
        'Value': [d, 34, 700, 0.600, 65, 62, 0, 200]
    }
    df = pd.DataFrame(data)

    tb = ax.table(cellText=df.values,
                  colLabels=None,
                  # colWidths=[0.35, 0.35],
                  bbox=[0, 0, 1, 1]
                )

    for i in range(len(df)):
        for j in range(3):
            cell = tb[(i, j)]
            cell.set_edgecolor('None')
            cell.PAD = 0

            if i % 2 == 1:
                cell.set_facecolor('#dcdcdc')
            if j == 0:
                cell.set_text_props(fontsize=24, fontstretch=1, horizontalalignment='left')
            elif j == 1:
                cell.set_text_props(fontsize=24, horizontalalignment='center', verticalalignment='center', linespacing=0)
            elif j == 2:
                cell.set_text_props(fontsize=24, horizontalalignment='right', verticalalignment='center', linespacing=0)

fig = plt.figure(figsize=(19.0, 10.0))
gs = gridspec.GridSpec(13, 23)

'''
ax_line_brain = plt.subplot(gs[0:3, 0:14])
ax_line_pnn20 = plt.subplot(gs[4:8, 0:14])
ax_line_pnn50 = plt.subplot(gs[9:13, 0:14])

ax_emo_map_pnn20 = plt.subplot(gs[0:6, 15:18])
ax_emo_map_pnn50 = plt.subplot(gs[0:6, 20:23])
ax_bio_table = plt.subplot(gs[7:13, 15:23])

'''

'''
ax_emo_map_pnn20 = plt.subplot(gs[1:6, 1:6])
# ax_emo_map_pnn50 = plt.subplot(gs[0:6, 5:8])
ax_bio_table = plt.subplot(gs[7:13, 0:7])

ax_line_brain = plt.subplot(gs[1:6, 9:15])
ax_line_pnn20 = plt.subplot(gs[1:6, 17:23])
ax_line_pnn50 = plt.subplot(gs[7:12, 9:15])
'''

ax_emo_map_pnn20 = plt.subplot(gs[1:6, 1:6])
ax_emo_map_pnn50 = plt.subplot(gs[8:13, 1:6])
ax_bio_table = plt.subplot(gs[7:13, 8:15])

ax_line_brain = plt.subplot(gs[1:6, 9:15])
ax_line_pnn = plt.subplot(gs[1:6, 17:23])
#ax_line_pnn50 = plt.subplot(gs[7:12, 9:15])


plt.subplots_adjust(left=0.05, right=0.95, bottom=0.05, top=0.95, wspace=0.4, hspace=0.4)

plot_emotion_map(ax_emo_map_pnn20)
plot_emotion_map(ax_emo_map_pnn50)
plot_bio_table(ax_bio_table)

plt.show()
