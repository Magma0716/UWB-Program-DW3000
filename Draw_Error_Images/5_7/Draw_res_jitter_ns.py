

import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns


# 非加密
data = {
    'Tags':          [1, 2, 2, 3, 3, 3, 4, 4, 4, 4],
    'Residual':      [0.0164, 0.0171, 0.0197, 0.0196, 0.0194, 0.0517, 0.0336, 0.0200, 0.0783, 0.0880],
    'Raw_Data_Jump': [0.0336, 0.0485, 0.0332, 0.0386, 0.0322, 0.0724, 0.0410, 0.0341, 0.1503, 0.0966],
    'TOF_Delay':     [2.29, 2.30, 2.29, 2.30, 2.33, 2.34, 2.34, 2.35, 2.37, 2.31]
}


# AES加密
'''data = {
    'Tags':          [1, 2, 2, 3, 3, 3, 4, 4, 4, 4],
    'Residual':      [0.0413, 0.0479, 0.0486, 0.0380, 0.0614, 0.0404, 0.0386, 0.1907, 0.1251, 0.1207],
    'Raw_Data_Jump': [0.0525, 0.0528, 0.0665, 0.0668, 0.0833, 0.1712, 0.0816, 0.2978, 0.2906, 0.1614],
    'TOF_Delay':     [2.27, 2.35, 2.43, 2.73, 2.32, 2.47, 3.04, 2.26, 2.42, 2.33]
}'''

df = pd.DataFrame(data)

# 2. 設定繪圖風格
sns.set_theme(style="whitegrid")
fig, axes = plt.subplots(1, 2, figsize=(15, 6))

# 左圖: residual & raw data jump
sns.lineplot(ax=axes[0], data=df, x='Tags', y='Raw_Data_Jump', marker='s', label='Raw_Data_Jump Avg', color='g')
sns.lineplot(ax=axes[0], data=df, x='Tags', y='Residual', marker='D', label='Residual Avg', color='b')
axes[0].set_title('Residual & Raw Data Jump (σ)', fontsize=14)
axes[0].set_xlabel('Number of Tags')
axes[0].set_ylabel('Value (m)')
axes[0].set_xticks([1, 2, 3, 4])
axes[0].legend()

# 右圖: tof delay
sns.lineplot(ax=axes[1], data=df, x='Tags', y='TOF_Delay', marker='o', label='TOF_Delay Avg', color='orange', errorbar='sd')
axes[1].set_title('TOF Delay (σ)', fontsize=14)
axes[1].set_xlabel('Number of Tags')
axes[1].set_ylabel('TOF Delay (ns)')
axes[1].set_xticks([1, 2, 3, 4])
axes[1].legend()

plt.tight_layout()
plt.show()


'''
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns

# 準備數據
data_none = {
    'Tags': [1, 2, 2, 3, 3, 3, 4, 4, 4, 4],
    'Residual': [0.0164, 0.0171, 0.0197, 0.0196, 0.0194, 0.0517, 0.0336, 0.0200, 0.0783, 0.0880],
    'Raw_Data_Jump': [0.0336, 0.0485, 0.0332, 0.0386, 0.0322, 0.0724, 0.0410, 0.0341, 0.1503, 0.0966],
    'TOF_Delay': [2.29, 2.30, 2.29, 2.30, 2.33, 2.34, 2.34, 2.35, 2.37, 2.31],
    'Type': 'Unencrypted Avg'
}

data_aes = {
    'Tags': [1, 2, 2, 3, 3, 3, 4, 4, 4, 4],
    'Residual': [0.0413, 0.0479, 0.0486, 0.0380, 0.0614, 0.0404, 0.0386, 0.1907, 0.1251, 0.1207],
    'Raw_Data_Jump': [0.0525, 0.0528, 0.0665, 0.0668, 0.0833, 0.1712, 0.0816, 0.2978, 0.2906, 0.1614],
    'TOF_Delay': [2.27, 2.35, 2.43, 2.73, 2.32, 2.47, 3.04, 2.26, 2.42, 2.33],
    'Type': 'AES Encrypted Avg'
}

df_all = pd.concat([pd.DataFrame(data_none), pd.DataFrame(data_aes)])

# 設定繪圖風格
sns.set_theme(style="whitegrid")
fig, axes = plt.subplots(1, 3, figsize=(22, 6))

palette_colors = {"Unencrypted Avg": "green", "AES Encrypted Avg": "red"}
line_styles = {"Unencrypted Avg": "", "AES Encrypted Avg": ""} # 可視需求調整虛實線

# Residual
sns.lineplot(ax=axes[0], data=df_all, x='Tags', y='Residual', hue='Type', 
             style='Type', markers=['s', 'D'], palette=palette_colors, errorbar='sd', linewidth=1, dashes=line_styles)
axes[0].set_title('Residual (σ)', fontsize=14)
axes[0].set_ylabel('Residual (m)')

# Raw Data Jump
sns.lineplot(ax=axes[1], data=df_all, x='Tags', y='Raw_Data_Jump', hue='Type', 
             style='Type', markers=['s', 'D'], palette=palette_colors, errorbar='sd', linewidth=1, dashes=line_styles)
axes[1].set_title('Raw Data Jump (σ)', fontsize=14)
axes[1].set_ylabel('Raw Data Jump (m)')

# TOF Delay
sns.lineplot(ax=axes[2], data=df_all, x='Tags', y='TOF_Delay', hue='Type', 
             style='Type', markers=['s', 'D'], palette=palette_colors, errorbar='sd', linewidth=1, dashes=line_styles)
axes[2].set_title('TOF Delay (σ)', fontsize=14)
axes[2].set_ylabel('Delay (ns)')

# 統一所有子圖的 X 軸
for ax in axes:
    ax.set_xticks([1, 2, 3, 4])
    ax.set_xlabel('Number of Tags')
    ax.legend()

plt.tight_layout()
plt.show()
'''