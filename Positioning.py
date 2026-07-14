import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 是否加入Tag?
Add_Tag = True

# 設定中文字型
plt.rcParams['font.sans-serif'] = ['Microsoft JhengHei']  # 微軟正黑體
plt.rcParams['axes.unicode_minus'] = False

# 定義錨點座標
anchors = {
    'Anchor 1': [0.00, 0.00, 0.00],
    'Anchor 2': [2.00, 0.00, 0.00],
    'Anchor 3': [1.00, 1.73, 0.00],
    'Anchor 4': [1.00, 0.00, 0.50]
}

# 計算標籤座標
pos1 = np.array(anchors['Anchor 1'])
pos2 = np.array(anchors['Anchor 2'])
pos3 = np.array(anchors['Anchor 3'])
tag1_pos = (pos1 + pos2 + pos3) / 3.0
tag1_pos = np.round(tag1_pos, 2) # 保留兩位小數

tags = {
    'Tag 1': [tag1_pos[0], tag1_pos[1], 0.1]
}

# 繪圖初始化
fig = plt.figure(figsize=(10, 8), dpi=300)
ax = fig.add_subplot(111, projection='3d')

x_coords = [pos[0] for pos in anchors.values()]
y_coords = [pos[1] for pos in anchors.values()]
z_coords = [pos[2] for pos in anchors.values()]

# 繪製錨點與標籤
ax.scatter(x_coords, y_coords, z_coords, c='blue', marker='s', s=180, edgecolors='black', depthshade=False, label='錨點(Anchor)')

if Add_Tag:
    tag_x_coords = [pos[0] for pos in tags.values()]
    tag_y_coords = [pos[1] for pos in tags.values()]
    tag_z_coords = [pos[2] for pos in tags.values()]
    ax.scatter(tag_x_coords, tag_y_coords, tag_z_coords, c='green', marker='o', s=180, edgecolors='black', depthshade=False, label='標籤(Tag)')

# 加入文字標籤
if Add_Tag:
    for name, pos in tags.items():
        label = f"{name}\n({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) m"
        ax.text(pos[0], pos[1], pos[2] - 0.1, label, color='green', fontsize=8, weight='bold', horizontalalignment='center')

for name, pos in anchors.items():
    label = f"{name}\n({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) m"
    ax.text(pos[0], pos[1], pos[2] + 0.05, label, color='black', fontsize=8, weight='bold', horizontalalignment='center')

# 繪製連線
all_points_positions = list(anchors.values())
for i, p1 in enumerate(all_points_positions):
    for j, p2 in enumerate(all_points_positions):
        if i < j:
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], c='#7f8c8d', linestyle='--', linewidth=1.2, alpha=0.7)

# 設定座標軸與視角
ax.set_xlim(min(x_coords) - 0.2, max(x_coords) + 0.2)
ax.set_xlabel('X-axis (unit: m)', fontsize=12)
ax.set_ylim(min(y_coords) - 0.2, max(y_coords) + 0.2)
ax.set_ylabel('Y-axis (unit: m)', fontsize=12)
ax.set_zlim(0, 0.70)
ax.set_zlabel('Z-axis (unit: m)', fontsize=12)
ax.view_init(elev=20, azim=60)

ax.grid(True, linestyle=':', alpha=0.6)
ax.legend(loc='upper right', fontsize=10)

# 輸出結果
plt.tight_layout()

if Add_Tag:
    plt.savefig('3D_Layout_with_1tag.png', dpi=300)
else:
    plt.savefig('3D_Layout.png', dpi=300)
    
plt.show()