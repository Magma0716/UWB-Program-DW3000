import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns

section = 1

if section == 0:

    choose = 1
    data = {}

    if choose == 0: # 非加密
        data = {
            'Tags':     [1, 2, 2, 3, 3, 3, 4, 4, 4, 4],
            'Residual': [0.0164, 0.0171, 0.0197, 0.0196, 0.0194, 0.0517, 0.0336, 0.0200, 0.0783, 0.0880],
            'Jitter':   [0.0336, 0.0485, 0.0332, 0.0386, 0.0322, 0.0724, 0.0410, 0.0341, 0.1503, 0.0966],
            'Latency':  [2.29, 2.30, 2.29, 2.30, 2.33, 2.34, 2.34, 2.35, 2.37, 2.31]
        }
    elif choose == 1: # AES加密
        data = {
            'Tags':     [1, 2, 2, 3, 3, 3, 4, 4, 4, 4],
            'Residual': [0.0221,    0.0339, 0.0486,    0.0380, 0.0570, 0.0945,    0.0386, 0.0811, 0.1181, 0.0827],
            'Jitter':   [0.0525,    0.0528, 0.0665,    0.0668, 0.0607, 0.1734,    0.0816, 0.2108, 0.1787, 0.1832],
            'Latency':  [2.25,      2.27, 2.43,        2.73, 2.32, 2.32,          3.04, 2.35, 2.42, 2.33]
        }

    df = pd.DataFrame(data)

    sns.set_theme(style="whitegrid")
    fig, axes = plt.subplots(1, 3, figsize=(15, 6))

    # 左圖: residual
    sns.lineplot(ax=axes[0], data=df, x='Tags', y='Residual', marker='D', label='Residual Average', color='b', errorbar=('pi', 100))
    sns.scatterplot(ax=axes[0], data=df, x='Tags', y='Residual', color='b', alpha=0.6)
    axes[0].set_title('Residual', fontsize=14)
    axes[0].set_xlabel('Number of Tags')
    axes[0].set_ylabel('Residual std (m)')
    axes[0].set_xticks([1, 2, 3, 4])
    axes[0].legend()

    # 中圖: jitter
    sns.lineplot(ax=axes[1], data=df, x='Tags', y='Jitter', marker='s', label='Jitter Average', color='g', errorbar=('pi', 100))
    sns.scatterplot(ax=axes[1], data=df, x='Tags', y='Jitter', color='g', alpha=0.6)
    axes[1].set_title('Jitter', fontsize=14)
    axes[1].set_xlabel('Number of Tags')
    axes[1].set_ylabel('Jitter std (m)')
    axes[1].set_xticks([1, 2, 3, 4])
    axes[1].legend()

    # 右圖: latency
    sns.lineplot(ax=axes[2], data=df, x='Tags', y='Latency', marker='o', label='Latency Average', color='orange', errorbar=('pi', 100))
    sns.scatterplot(ax=axes[2], data=df, x='Tags', y='Latency', color='orange', alpha=0.6)
    axes[2].set_title('Latency', fontsize=14)
    axes[2].set_xlabel('Number of Tags')
    axes[2].set_ylabel('Latency std (ns)')
    axes[2].set_xticks([1, 2, 3, 4])
    axes[2].legend()

    plt.tight_layout()
    plt.show()

# =================================

elif section == 1:

    # 準備數據
    data_none = {
        'Tags': [1, 2, 2, 3, 3, 3, 4, 4, 4, 4],
        'Residual': [0.0164, 0.0171, 0.0197, 0.0196, 0.0194, 0.0517, 0.0336, 0.0200, 0.0783, 0.0880],
        'Jitter': [0.0336, 0.0485, 0.0332, 0.0386, 0.0322, 0.0724, 0.0410, 0.0341, 0.1503, 0.0966],
        'Latency': [2.29, 2.30, 2.29, 2.30, 2.33, 2.34, 2.34, 2.35, 2.37, 2.31],
        'Type': 'Unencrypted Avg'
    }

    data_aes = {
       'Tags':     [1, 2, 2, 3, 3, 3, 4, 4, 4, 4],
            'Residual': [0.0221,    0.0339, 0.0486,    0.0380, 0.0570, 0.0945,    0.0386, 0.0811, 0.1181, 0.0827],
            'Jitter':   [0.0525,    0.0528, 0.0665,    0.0668, 0.0607, 0.1734,    0.0816, 0.2108, 0.1787, 0.1832],
            'Latency':  [2.25,      2.27, 2.43,        2.73, 2.32, 2.32,          3.04, 2.35, 2.42, 2.33],
        'Type': 'AES Encrypted Avg'
    }

    df_all = pd.concat([pd.DataFrame(data_none), pd.DataFrame(data_aes)])

    # 設定繪圖風格
    sns.set_theme(style="whitegrid")
    fig, axes = plt.subplots(1, 3, figsize=(22, 6))

    palette_colors = {"Unencrypted Avg": "green", "AES Encrypted Avg": "red"}
    line_styles    = {"Unencrypted Avg": "", "AES Encrypted Avg": ""}

    # Residual
    sns.lineplot(ax=axes[0], data=df_all, x='Tags', y='Residual', hue='Type', style='Type', markers=['s', 'D'], palette=palette_colors, errorbar=('pi', 100), linewidth=1, dashes=line_styles)
    sns.scatterplot(ax=axes[0], data=data_none, x='Tags', y='Residual', color='g', alpha=0.6)
    sns.scatterplot(ax=axes[0], data=data_aes, x='Tags', y='Residual', color='r', alpha=0.6)
    axes[0].set_title('Residual', fontsize=14)
    axes[0].set_ylabel('Residual std (m)')

    # Jitter
    sns.lineplot(ax=axes[1], data=df_all, x='Tags', y='Jitter', hue='Type', style='Type', markers=['s', 'D'], palette=palette_colors, errorbar=('pi', 100), linewidth=1, dashes=line_styles)
    sns.scatterplot(ax=axes[1], data=data_none, x='Tags', y='Jitter', color='g', alpha=0.6)
    sns.scatterplot(ax=axes[1], data=data_aes, x='Tags', y='Jitter', color='r', alpha=0.6)
    axes[1].set_title('Jitter', fontsize=14)
    axes[1].set_ylabel('Jitter std (m)')

    # Latency
    sns.lineplot(ax=axes[2], data=df_all, x='Tags', y='Latency', hue='Type', style='Type', markers=['s', 'D'], palette=palette_colors, errorbar=('pi', 100), linewidth=1, dashes=line_styles)
    sns.scatterplot(ax=axes[2], data=data_none, x='Tags', y='Latency', color='g', alpha=0.6)
    sns.scatterplot(ax=axes[2], data=data_aes, x='Tags', y='Latency', color='r', alpha=0.6)
    axes[2].set_title('Latency', fontsize=14)
    axes[2].set_ylabel('Latency std (ns)')

    for ax in axes:
        ax.set_xticks([1, 2, 3, 4])
        ax.set_xlabel('Number of Tags')
        ax.legend()

    plt.tight_layout()
    plt.show()