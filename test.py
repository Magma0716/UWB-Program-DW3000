import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
import sys

# point
ms_point = {
    -1: [34.0, 35.5, 36.1, 37.6, 34.4, 35.6, 36, 36.9],
    0:  [40.8, 39.7, 43.7, 40.5, 43.0, 39.9, 39.6, 44.8, 39.9],
    1:  [45.5, 42.8, 43.6, 44.4,   40],
    2:  [39.5, 41.4, 44.3, 39.4],
    4:  [37.9, 43.1, 41.3, 38.9],
    8:  [37.4, 40.9, 44.3, 41.9],
    16: [39.4, 44.1, 42.1, 45.4, 42.3, 41.2],
    24: [43.5, 39.8, 42.8, 41.7],
    32: [42.1, 41.3, 39.7, 44.5, 40.4],
    47: [40.9, 46.9, 44.3, 42.4]
}

dis_point = {
    -1: [0.0695, 0.0408, 0.0702, 0.0483, 0.0900, 0.1126, 0.0515, 0.0491],
    0:  [0.0632, 0.0967, 0.0738, 0.0521, 0.0755, 0.0708, 0.0656, 0.1127, 0.0609],
    1:  [0.0734, 0.0731, 0.0621, 0.0592,  0.066],
    2:  [0.0788, 0.0524, 0.0677, 0.0682],
    4:  [0.0750, 0.1807, 0.0593, 0.0619],
    8:  [0.0864, 0.0750, 0.0504, 0.0534],
    16: [0.0854, 0.0735, 0.1015, 0.0736, 0.0899, 0.0641],
    24: [0.0642, 0.0702, 0.0539, 0.0779],
    32: [0.0873, 0.1324, 0.0915, 0.0616, 0.0390],
    47: [0.0713, 0.0582, 0.0593, 0.0737]
}

# 數值

# [35.5, 39.7, 40, 41.4, 41.3, 41.9, 42.3, 43.5, 44.5, 46.9]
padding = [-1, 0, 1, 2, 4, 8, 16, 24, 32, 44]
ms = [35.5, 39.7, 40, 39.5, 41.3, 40.9, 41.2, 41.7, 42.1, 44.3]
dis = []

for i in range(len(padding)):
    index = ms_point[padding[i]].index(ms[i])
    dis.append(dis_point[padding[i]][index])

# raw to dataframe
def dict_to_df(data_dict, value_name):
    rows = []
    for p, vals in data_dict.items():
        for v in vals:
            rows.append({'padding': p, value_name: v})
    return pd.DataFrame(rows)

ms_point = dict_to_df(ms_point, 'ms_point')
dis_point = dict_to_df(dis_point, 'dis_point')

df = pd.DataFrame({
    'padding': padding[:len(ms)],
    'ms_std': ms,
    'dis_std': dis
})

sns.set_theme(style="whitegrid")
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))

# 圖 1 (ms)
sns.lineplot(data=df, x='padding', y='ms_std', marker='o', ax=ax1, color='darkorange', markersize=8, label='Trend')
sns.scatterplot(data=ms_point, x='padding', y='ms_point', ax=ax1, color='darkorange', alpha=0.4, s=50, label='Raw Data')

ax1.set_title('Padding vs. ms(σ)', fontsize=14, fontweight='bold')
ax1.set_xlabel('Padding', fontsize=12)
ax1.set_ylabel('ms(σ)', fontsize=12)
ax1.set_xticks(padding) 
ax1.legend(loc='lower right')

# 圖 2 (dis)
sns.lineplot(data=df, x='padding', y='dis_std', marker='s', ax=ax2, color='royalblue', markersize=8, label='Trend')
sns.scatterplot(data=dis_point, x='padding', y='dis_point', ax=ax2, color='royalblue', alpha=0.4, s=50, label='Raw Data')

ax2.set_title('Padding vs. Distance(σ)', fontsize=14, fontweight='bold')
ax2.set_xlabel('Padding', fontsize=12)
ax2.set_ylabel('Distance(σ)', fontsize=12)
ax2.set_xticks(padding) 
ax2.legend()

plt.tight_layout()
plt.show()