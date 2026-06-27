import socket
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
from typing import Dict, Tuple
import cmath

# 卡爾曼濾波器 (EKF)
class UWB_EKF:
    def __init__(self, x0=0, y0=0, dt=0.016):
        # 狀態向量 [x, y, vx, vy]
        self.X = np.array([[x0], [y0], [0], [0]], dtype=float)
        
        # 狀態轉移矩陣 A
        self.dt = dt
        self.A = np.array([
            [1, 0, self.dt, 0],
            [0, 1, 0, self.dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        self.P = np.eye(4) * 1.0    # 預測誤差協方差 P
        self.Q = np.eye(4) * 0.005  # 過程雜訊 Q  <--控制平滑度
        self.R_val = 0.1            # 測量雜訊 R  <--UWB距離誤差 (default: 0.1m)

    def predict(self):
        self.X = np.dot(self.A, self.X)                             # 預測下個位置 (A * X)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q  # 更新預測誤差 (A * P * A.T + Q)

    def update(self, distances, anchor_pos):
        """
        distances: {'A1': dist1, ...}
        anchor_pos: {'A1': (x, y), ...}
        """
        for aid, z_meas in distances.items():
            if aid not in anchor_pos: continue
            
            an_x, an_y = anchor_pos[aid]
            
            # 計算預期測量值 (當前估計點到 Anchor 的幾何距離)
            dx = self.X[0, 0] - an_x
            dy = self.X[1, 0] - an_y
            z_pred = np.sqrt(dx**2 + dy**2) # 距離函數
            
            if z_pred < 0.001: z_pred = 0.001 # 防除以0
            
            # 觀測矩陣 H (Jacobian matrix)
            H = np.array([[dx/z_pred, dy/z_pred, 0, 0]]) # 距離函數對 [x, y, vx, vy] 的偏微分
            
            # 卡爾曼增益 K
            S = np.dot(np.dot(H, self.P), H.T) + self.R_val
            K = np.dot(np.dot(self.P, H.T), np.linalg.inv(S))
            
            y_error = z_meas - z_pred                           # 測量殘差
            self.X = self.X + np.dot(K, y_error)                # 更新狀態 X
            self.P = np.dot((np.eye(4) - np.dot(K, H)), self.P) # 誤差 P

    def get_state(self):
        # 移動速度
        speed = np.sqrt(self.X[2, 0]**2 + self.X[3, 0]**2)
        return self.X[0, 0], self.X[1, 0], speed

# 定位系統
class UWBPositionSystem:
    
    def __init__(self):
        # 設定 UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('192.168.0.108', 8001))
        self.sock.setblocking(False)
        
        # Anchor 位置
        self.distance_A1_A2 = 2.0  # <-- 需要修改
        self.anchor_positions = {
            'A1': (0.0, 0.0),
            'A2': (self.distance_A1_A2, 0.0),
            'A3': (self.distance_A1_A2 / 2.0, self.distance_A1_A2),
        }

        # 按鈕
        self.show_ekf = True
        self.show_circles = True
        self.show_raw = True

        # 卡爾曼濾波器 EKF
        self.ekf = UWB_EKF(dt=0.016)

        # 圖表設定
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        plt.subplots_adjust(bottom=0.2)
        self.limit_x = [-2, 6]
        self.limit_y = [-2, 6]
        self.ax.set_xlim(self.limit_x)
        self.ax.set_ylim(self.limit_y)
        self.ax.set_aspect('equal')
        self.ax.grid(True, linestyle='--', alpha=0.5)
        self.ax.set_title('DW3000 Tracking (with EKF enhanced)')

        # Anchor 圈圈
        self.circles = {
            'A1': plt.Circle(self.anchor_positions['A1'], 0, color='#a9fbab', fill=False, lw=1.5, visible=False),
            'A2': plt.Circle(self.anchor_positions['A2'], 0, color='#a9fbab', fill=False, lw=1.5, visible=False),
            'A3': plt.Circle(self.anchor_positions['A3'], 0, color='#a9fbab', fill=False, lw=1.5, visible=False),
        }
        for c in self.circles.values(): self.ax.add_artist(c)
        
        # Anchor 連線
        self.lines = {
            'L12': self.ax.plot([], [], 'b-')[0],
            'L23': self.ax.plot([], [], 'b-')[0],
            'L31': self.ax.plot([], [], 'b-')[0],
        }
        
        # Anchor 點
        for aid, pos in self.anchor_positions.items():
            self.ax.plot(pos[0], pos[1], 'bo', markersize=8)
            self.ax.text(pos[0], pos[1]-0.3, aid, ha='center')
        
        # Anchor 靜態標記
        self.anchor_dots = []
        for aid, pos in self.anchor_positions.items():
            dot, = self.ax.plot(pos[0], pos[1], 'bo', markersize=8, zorder=10)
            self.ax.text(pos[0], pos[1]-0.3, aid, ha='center', zorder=10)
            self.anchor_dots.append(dot)
        
        # label
        self.tag_point, = self.ax.plot([], [], 'ro', markersize=10, label='EKF Tag')
        self.raw_point, = self.ax.plot([], [], 'kx', markersize=7,  label='Raw Tag', alpha=0.4)
        
        self.tag_label =  self.ax.text(0, 0, '', fontsize=10, fontweight='bold', color='red')
        self.tag_trail, = self.ax.plot([], [], 'r-', alpha=0.5, linewidth=1)
        self.trail_x, self.trail_y = [], []
        self.ax.legend(loc='upper right')

        # 按鈕
        self.create_buttons()
    
    def create_buttons(self):
        ax_raw = plt.axes([0.15, 0.05, 0.2, 0.06])
        ax_cir = plt.axes([0.40, 0.05, 0.2, 0.06])
        ax_ekf = plt.axes([0.65, 0.05, 0.2, 0.06])

        self.btn_raw = Button(ax_raw, 'Raw: ON', color='#bdc3c7', hovercolor='#95a5a6')
        self.btn_cir = Button(ax_cir, 'Circles: ON', color='#2ecc71', hovercolor='#27ae60')
        self.btn_ekf = Button(ax_ekf, 'EKF: ON', color='#e74c3c', hovercolor='#c0392b')

        self.btn_raw.on_clicked(lambda e: self.toggle('raw'))
        self.btn_cir.on_clicked(lambda e: self.toggle('cir'))
        self.btn_ekf.on_clicked(lambda e: self.toggle('ekf'))
    
    def toggle(self, mode):
        if mode == 'raw':
            self.show_raw = not self.show_raw
            self.btn_raw.label.set_text(f"Raw: {'ON' if self.show_raw else 'OFF'}")
        elif mode == 'cir':
            self.show_circles = not self.show_circles
            self.btn_cir.label.set_text(f"Circles: {'ON' if self.show_circles else 'OFF'}")
        elif mode == 'ekf':
            self.show_ekf = not self.show_ekf
            self.btn_ekf.label.set_text(f"EKF: {'ON' if self.show_ekf else 'OFF'}")
    
    def pos2(self, d1, d2):
        dist = self.distance_A1_A2
        try:
            cosA = (d1**2 + dist**2 - d2**2) / (2 * d1 * dist)
            cosA = max(-1.0, min(1.0, cosA))
            
            x = d1 * cosA
            y = d1 * cmath.sqrt(1 - cosA**2).real
            return x, y
        except:
            return None, None

    def pos3(self, distances: Dict[str, float]) -> Tuple[float, float]:
        if len(distances) < 3: return None, None
        A, b = [], []
        ids = ['A1', 'A2', 'A3'] # list(distances.keys())[:3]
        for i in range(2):
            p1, p2 = self.anchor_positions[ids[i]], self.anchor_positions[ids[i+1]]
            r1, r2 = distances[ids[i]], distances[ids[i+1]]
            A.append([2 * (p2[0] - p1[0]), 2 * (p2[1] - p1[1])])
            b.append(r1**2 - r2**2 - p1[0]**2 - p1[1]**2 + p2[0]**2 + p2[1]**2)
        try:
            return np.linalg.solve(A, b)
        except:
            return None, None

    def update_plot(self, frame):    
        latest_data = None
        
        while True:
            try:
                data, _ = self.sock.recvfrom(2048)
                latest_data = data
            except: break
            
        if latest_data:
            # 隱藏狀態    
            for c in self.circles.values(): c.set_visible(False)
            for l in self.lines.values(): l.set_data([], [])
            
            try:
                json_data = json.loads(latest_data.decode())
                dists = {arr['id']: max(0.01, arr['distance']) for arr in json_data['anchors']}
                raw_x, raw_y = None, None
                num = len(dists)

                print(json_data)
                
                # raw tag
                if num >= 1 and 'A1' in dists:
                    self.circles['A1'].set_radius(dists['A1'])
                    self.circles['A1'].set_visible(self.show_circles)
                    if num == 1: raw_x, raw_y = dists['A1'], 0.0

                if num >= 2 and 'A1' in dists and 'A2' in dists:
                    self.circles['A2'].set_radius(dists['A2'])
                    self.circles['A2'].set_visible(self.show_circles)
                    
                    self.lines['L12'].set_data([0, self.distance_A1_A2], [0, 0])
                    if num == 2: raw_x, raw_y = self.pos2(dists['A1'], dists['A2'])

                if num >= 3 and 'A1' in dists and 'A2' in dists and 'A3' in dists:
                    self.circles['A3'].set_radius(dists['A3'])
                    self.circles['A3'].set_visible(self.show_circles)
                    
                    p1, p2, p3 = self.anchor_positions['A1'], self.anchor_positions['A2'], self.anchor_positions['A3']
                    self.lines['L23'].set_data([p2[0], p3[0]], [p2[1], p3[1]])
                    self.lines['L31'].set_data([p3[0], p1[0]], [p3[1], p1[1]])
                    raw_x, raw_y = self.pos3(dists)

                # raw point
                if raw_x is not None and self.show_raw:
                    self.raw_point.set_data([raw_x], [raw_y])
                else:
                    self.raw_point.set_data([], [])
                
                # EKF point
                self.ekf.predict()
                self.ekf.update(dists, self.anchor_positions)
                sx, sy, speed = self.ekf.get_state()

                # 邊界與顯示
                if self.show_ekf:
                    sx = max(self.limit_x[0], min(self.limit_x[1], sx))
                    sy = max(self.limit_y[0], min(self.limit_y[1], sy))
                    self.tag_point.set_data([sx], [sy])
                    self.tag_label.set_position((sx + 0.2, sy + 0.2))
                    self.tag_label.set_text(f"({sx:.2f}, {sy:.2f}) {speed:.2f} m/s")
                    
                    self.trail_x.append(sx); self.trail_y.append(sy)
                    if len(self.trail_x) > 40: self.trail_x.pop(0); self.trail_y.pop(0)
                    self.tag_trail.set_data(self.trail_x, self.trail_y)
                else:
                    self.tag_point.set_data([], []); self.tag_trail.set_data([], [])
                    self.tag_label.set_text("")
                
            except Exception as e:
                print(f"Update Error: {e}")

        return(
            self.tag_point, self.raw_point, self.tag_label, self.tag_trail, 
            *self.circles.values(), *self.lines.values(), *self.anchor_dots
        )

    def run(self):
        # interval 16ms (60fps)
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=16, blit=True, cache_frame_data=False)
        plt.show()

if __name__ == "__main__":
    UWBPositionSystem().run()