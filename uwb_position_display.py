import socket
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from typing import Dict, Tuple
import cmath

class UWBPositionSystem:
    def __init__(self):
        # 設定 UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('192.168.0.108', 8001))
        self.sock.settimeout(0.1)
        self.sock.setblocking(False)
        
        # Anchor 位置設定 (x, y) in meters
        self.anchor_nums = 2
        self.distance_A1_A2 = 2.0
        self.anchor_positions = {
            'A1': (0.0, 0.0),
            'A2': (2.0, 0.0),
            'A3': (1.0, 2.0),
        }
 
        # 圖表
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.ax.set_xlim(-1, 9)
        self.ax.set_ylim(-1, 7)
        self.ax.grid(False)
        self.ax.set_title('UWB Position Tracking')
        self.ax.set_xlabel('X Position (meters)')
        self.ax.set_ylabel('Y Position (meters)')

        self.range_circle = plt.Circle((0, 0), 0, color='r', fill=False, linestyle='-', alpha=0.3)
        self.c1 = plt.Circle((0, 0), 0, color='g', fill=False, linestyle=':', alpha=0.2)
        self.c2 = plt.Circle((self.distance_A1_A2, 0), 0, color='g', fill=False, linestyle=':', alpha=0.2)
        self.ax.add_artist(self.range_circle)
        self.ax.add_artist(self.c1)
        self.ax.add_artist(self.c2)
        
        # 畫出 Anchor 位置
        for anchor_id, pos in self.anchor_positions.items():
            self.ax.plot(pos[0], pos[1], 'bs', markersize=10)
            self.ax.text(
                pos[0], pos[1] - 0.3, anchor_id, fontsize=10, 
                fontweight='bold', ha='center', color='blue'
            )
        self.ax.legend()

        # Tag 位置點
        self.tag_point, = self.ax.plot([], [], 'ro', markersize=10, label='Tag')
        self.tag_label = self.ax.text(0, 0, '', fontsize=10, fontweight='bold', color='red')
        self.tag_trail, = self.ax.plot([], [], 'r-', alpha=0.5)  # 軌跡線
        self.trail_x, self.trail_y = [], []
        self.max_trail_points = 50  # 最多顯示50個歷史點

        # 距離文字顯示
        self.distance_texts = {}
        for anchor_id in self.anchor_positions:
            text = self.ax.text(0, 0, '', fontsize=8)
            self.distance_texts[anchor_id] = text

    # 公式
    def pos2(self, d1, d2):
        """餘弦定理定位"""
        dist = self.distance_A1_A2
        if d1 == 0 or dist == 0: return 0.0, 0.0
        
        try:
            cosA = (d1**2 + dist**2 - d2**2) / (2 * d1 * dist)
            cosA = max(-1.0, min(1.0, cosA))
            
            x = d1 * cosA
            y = d1 * cmath.sqrt(1 - cosA**2).real
            
            return round(x.real, 2), round(y.real, 2)
        
        except Exception as e:
            return 0.0, 0.0

    def pos3(self, distances: Dict[str, float]) -> Tuple[float, float]:
        """三邊測量法"""
        if len(distances) < 3:
            return None, None

        # 準備方程式係數
        A = []
        b = []

        # 選擇前三個 anchors
        anchor_ids = list(distances.keys())[:3]

        for i in range(2):  # 我們只需要兩個方程式
            anchor1 = self.anchor_positions[anchor_ids[i]]
            anchor2 = self.anchor_positions[anchor_ids[i+1]]
            r1 = distances[anchor_ids[i]]
            r2 = distances[anchor_ids[i+1]]

            # 建立方程式係數
            A.append([
                2 * (anchor2[0] - anchor1[0]),
                2 * (anchor2[1] - anchor1[1])
            ])

            b.append(
                r1*r1 - r2*r2 -
                anchor1[0]*anchor1[0] - anchor1[1]*anchor1[1] +
                anchor2[0]*anchor2[0] + anchor2[1]*anchor2[1]
            )

        try:
            # 解聯立方程式
            position = np.linalg.solve(A, b)
            return position[0], position[1]
        except np.linalg.LinAlgError:
            return None, None

    # 圖表
    def update_plot(self, frame):
        """更新圖表"""
        latest_data = None
        while True:
            
            # 接收 UDP 數據
            try:
                data, addr = self.sock.recvfrom(1024)
                latest_data = data
            except socket.error:
                break
            
        if latest_data:
            try:
                json_data = json.loads(data.decode())
                
                # 解析距離數據
                distances = {arr['id']: arr['distance'] for arr in json_data['anchors']}
                print(distances)
                
                x, y = None, None
                
                # 確認 anchor 數量
                self.anchor_nums = len(distances)
                
                if self.anchor_nums == 1 and 'A1' in distances:
                    x, y = distances['A1'], 0.0
                    self.range_circle.center = self.anchor_positions['A1']
                    self.range_circle.set_radius(distances['A1'])
                
                elif self.anchor_nums == 2 and 'A1' in distances and 'A2' in distances:
                    d1, d2 = distances['A1'], distances['A2']
                    x, y = self.pos2(d1, d2)
                    self.c1.set_radius(d1)
                    self.c2.set_radius(d2)
                
                elif self.anchor_nums >= 3:
                    x, y = self.pos3(distances)

                if x is not None and y is not None:
                    # 更新點的位置
                    self.tag_point.set_data([x], [y])
                    # 更新標籤位置 (稍微偏移一點)
                    self.tag_label.set_position((x + 0.2, y + 0.2))
                    self.tag_label.set_text(f"Tag ({x:.2f}, {y:.2f})")
                    
                    # 更新軌跡
                    self.trail_x.append(x)
                    self.trail_y.append(y)
                    if len(self.trail_x) > self.max_trail_points:
                        self.trail_x.pop(0)
                        self.trail_y.pop(0)
                    self.tag_trail.set_data(self.trail_x, self.trail_y)
                
            except Exception as e:
                print(f"Error: {e}")

        return (self.tag_point, self.tag_label, self.tag_trail, self.range_circle, self.c1, self.c2)

    # 動畫
    def run(self):
        """運行動畫"""
        ani = FuncAnimation(
            self.fig, self.update_plot, interval=16,
            blit=True, cache_frame_data=False
        )
        plt.show()

    def update_tag_visuals(self, x, y, distances):
        """抽離出來的更新視覺化元件函式"""
        self.tag_point.set_data([x], [y])
        self.trail_x.append(x)
        self.trail_y.append(y)
        if len(self.trail_x) > self.max_trail_points:
            self.trail_x.pop(0)
            self.trail_y.pop(0)
        self.tag_trail.set_data(self.trail_x, self.trail_y)
        
        # 更新距離文字
        for anchor_id, dist in distances.items():
            if anchor_id in self.distance_texts:
                a_pos = self.anchor_positions[anchor_id]
                self.distance_texts[anchor_id].set_position(((a_pos[0]+x)/2, (a_pos[1]+y)/2))
                self.distance_texts[anchor_id].set_text(f'{anchor_id}: {dist:.2f}m')
    
if __name__ == "__main__":
    system = UWBPositionSystem()
    system.run()