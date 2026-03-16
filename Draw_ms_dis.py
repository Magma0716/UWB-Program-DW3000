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
    if selected_points:
        # 按 Padding 排序，確保連線從左到右
        selected_points.sort()
        
        # 取得目前所有選中點的座標
        h_x = [p for p, i in selected_points]
        h_y1 = [ms_raw[p][i] for p, i in selected_points]
        h_y2 = [dis_raw[p][i] for p, i in selected_points]
        
        # 更新亮點
        highlights1.set_data(h_x, h_y1)
        highlights2.set_data(h_x, h_y2)
        
        # --- 修正後的連線邏輯 ---
        # 如果同一個 Padding 有多個點，我們取該 Padding 下點選點的平均值來連線
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
        highlights1.set_data([], [])
        highlights2.set_data([], [])
        line1.set_data([], [])
        line2.set_data([], [])

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