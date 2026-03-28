import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import json

with open('AES_RandomIV_Results.json','r') as file:
    data = json.load(file)

# --- 原始資料 (保持不變) ---
ms_raw = {int(k): v for k, v in data['ms_raw'].items()}
dis_raw = {int(k): v for k, v in data['dis_raw'].items()}

print(ms_raw, dis_raw)

padding_list = sorted(ms_raw.keys())
selected_points = []  # 格式: (padding, index)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
plt.subplots_adjust(hspace=0.3)

# 1. 趨勢線層 (與點連線)
line1, = ax1.plot([], [], color='darkorange', linewidth=2, zorder=1, alpha=0.8, marker='')
line2, = ax2.plot([], [], color='royalblue', linewidth=2, zorder=1, alpha=0.8, marker='')

# 2. 亮點層 (選中後的視覺效果)
highlights1, = ax1.plot([], [], 'o', color='darkorange', markersize=8, zorder=3)
highlights2, = ax2.plot([], [], 's', color='royalblue', markersize=8, zorder=3)

# 3. 背景散點
for p in padding_list:
    ax1.scatter([p]*len(ms_raw[p]), ms_raw[p], color='gray', alpha=0.15, s=50, picker=True, gid=p)
    ax2.scatter([p]*len(dis_raw[p]), dis_raw[p], color='gray', alpha=0.15, s=50, picker=True, gid=p)

def update_plot():
    # 當選中點發生變化時，清空終端機螢幕並印出最新列表
    import os
    # 根據不同作業系統清空終端機 (Windows 是 cls, Linux/Mac 是 clear)
    # 如果你不需要清空螢幕，可以註解掉下面這行
    os.system('cls') 

    print("\n" + "="*50)
    print(f"{'Padding':<15} | {'ns(σ)':<15} | {'Distance(σ)':<15}")
    print("-"*50)

    if selected_points:
        # 按 Padding 排序，確保顯示從左到右
        selected_points.sort()
        
        h_x = [p for p, i in selected_points]
        h_y1 = [ms_raw[p][i] for p, i in selected_points]
        h_y2 = [dis_raw[p][i] for p, i in selected_points]
        
        # 逐點列印目前選擇的數值
        for p, y1, y2 in zip(h_x, h_y1, h_y2):
            print(f"{p:<15} | {y1:<15.4f} | {y2:<15.4f}")

        # 更新亮點圖層
        highlights1.set_data(h_x, h_y1)
        highlights2.set_data(h_x, h_y2)
        
        # 連線邏輯 (取平均)
        unique_paddings = sorted(list(set(h_x)))
        if len(unique_paddings) > 1:
            line_x = unique_paddings
            line_y1 = [np.mean([ms_raw[p][i] for pp, i in selected_points if pp == p]) for p in unique_paddings]
            line_y2 = [np.mean([dis_raw[p][i] for pp, i in selected_points if pp == p]) for p in unique_paddings]
            
            line1.set_data(line_x, line_y1)
            line2.set_data(line_x, line_y2)
        else:
            line1.set_data([], [])
            line2.set_data([], [])
    else:
        print("尚未選取任何點。")
        highlights1.set_data([], [])
        highlights2.set_data([], [])
        line1.set_data([], [])
        line2.set_data([], [])

    print("="*50)
    ax1.set_title(f"Interactive Tracking | Selected Points: {len(selected_points)}")
    fig.canvas.draw_idle()

def on_pick(event):
    p_id = event.artist.get_gid()
    if p_id is None: return

    # 獲取點擊點在該 Padding 中的索引
    ind = event.ind[0]
    point_id = (p_id, ind)

    if point_id in selected_points:
        selected_points.remove(point_id)
    else:
        selected_points.append(point_id)

    update_plot()

# 樣式設定
ax1.set_ylabel('ns(σ)')
ax1.set_xticks(padding_list)
ax2.set_ylabel('Distance(σ)')
ax2.set_xlabel('Padding Size')
ax2.set_xticks(padding_list)

fig.canvas.mpl_connect('pick_event', on_pick)
plt.show()