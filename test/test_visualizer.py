import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib import patches
import pandas as pd

def plot_emotion_map(ax):
    ax.set_aspect('equal')
    ax.set_ylim([-4, 4])
    ax.set_xlim([-4, 4])
    ax.set_xticks([])
    ax.set_yticks([])

    ax.axhline(0, color='k', lw=0.5)
    ax.axline((0, 0), slope=1.0, color='k', lw=0.5)
    ax.axvline(0, color='k', lw=0.5)
    ax.axline((0, 0), slope=-1.0, color='k', lw=0.5)

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

    for (x, y), label in zip(positions, labels):
        ax.text(x, y, label, ha='center', va='center', size=15)

    theta = np.deg2rad(30)
    wedge = plt.Polygon([[0, 0], [np.cos(theta), np.sin(theta)], [1, 0]], closed=True, color='yellow', alpha=0.5)
    ax.add_artist(wedge)

    ax.plot([0, np.cos(theta)], [0, np.sin(theta)], 'r-', lw=2)
    ax.plot([0, 1], [0, 0], 'g-', lw=2)
    ax.plot(np.cos(theta), np.sin(theta), 'ko')

def plot_bio_table(ax):
    ax.axis('off')
    ax.axis('tight')

    data = {
        'Label': ['Timestamp', 'BPM', 'IBI', 'pNN20', 'pNN50', 'Attention', 'Meditation', 'poorsignal'],
        'Character': [':', ':', ':', ':', ':', ':', ':', ':'],
        'Value': ['12:12:13', 34, 700, 0.600, 65, 62, 0, 200]
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
