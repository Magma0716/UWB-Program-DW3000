import socket
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button

# ==========================================
# 核心演算法：擴展卡爾曼濾波器 (EKF)
# ==========================================
class UWB_EKF:
    def __init__(self, x0=0, y0=0, dt=0.016):
        self.X = np.array([[x0], [y0], [0], [0]], dtype=float)
        self.dt = dt
        self.A = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        self.P = np.eye(4) * 1.0  
        self.Q = np.eye(4) * 0.005 
        self.R_val = 0.08         

    def predict(self):
        self.X = np.dot(self.A, self.X)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q

    def update(self, distances, anchor_pos):
        for aid, z_meas in distances.items():
            if aid not in anchor_pos: continue
            an_x, an_y = anchor_pos[aid]
            dx, dy = self.X[0,0] - an_x, self.X[1,0] - an_y
            z_pred = np.sqrt(dx**2 + dy**2)
            if z_pred < 0.001: z_pred = 0.001
            
            H = np.array([[dx/z_pred, dy/z_pred, 0, 0]])
            S = np.dot(np.dot(H, self.P), H.T) + self.R_val
            K = np.dot(np.dot(self.P, H.T), np.linalg.inv(S))
            
            self.X = self.X + np.dot(K, (z_meas - z_pred))
            self.P = np.dot((np.eye(4) - np.dot(K, H)), self.P)

    def get_state(self):
        speed = np.sqrt(self.X[2, 0]**2 + self.X[3, 0]**2)
        return self.X[0, 0], self.X[1, 0], speed

# ==========================================
# 終極 UWB 定位系統
# ==========================================
class UWBPositionSystem:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('192.168.0.108', 8001))
        self.sock.setblocking(False)
        
        self.anchor_positions = {
            'A1': (0.0, 0.0),
            'A2': (2.0, 0.0),
            'A3': (1.0, 2.0),
        }

        self.show_ekf = True
        self.show_circles = True
        self.show_raw = True
        self.ekf = UWB_EKF(dt=0.016)

        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        plt.subplots_adjust(bottom=0.2) 
        
        # 固定範圍，避免 Anchor 跑掉
        self.limit_x = [-2, 6]
        self.limit_y = [-2, 6]
        self.ax.set_xlim(self.limit_x)
        self.ax.set_ylim(self.limit_y)
        self.ax.set_aspect('equal')
        self.ax.grid(True, linestyle=':', alpha=0.6)
        self.ax.set_title("UWB Ultimate Tracking (EKF)", fontsize=14)

        # 1. 距離圓圈
        self.circles = {aid: plt.Circle(pos, 0, color='#2ecc71', fill=False, lw=1, alpha=0.5, visible=False) 
                        for aid, pos in self.anchor_positions.items()}
        for c in self.circles.values(): self.ax.add_artist(c)

        # 2. 原始點 (Raw)
        self.raw_point, = self.ax.plot([], [], 'kx', markersize=8, alpha=0.5, label='Raw Meas.')

        # 3. EKF 點與軌跡
        self.tag_point, = self.ax.plot([], [], 'ro', markersize=10, label='EKF Filtered', zorder=5)
        self.tag_trail, = self.ax.plot([], [], 'r-', alpha=0.3, lw=2, zorder=4)
        self.trail_x, self.trail_y = [], []
        
        # 4. Anchor 點 (強制定在最上層 zorder=10)
        self.anchor_plots = []
        for aid, pos in self.anchor_positions.items():
            p, = self.ax.plot(pos[0], pos[1], 'bo', markersize=10, zorder=10)
            self.ax.text(pos[0], pos[1]-0.4, aid, ha='center', fontweight='bold', color='blue', zorder=10)
            self.anchor_plots.append(p)
        
        self.tag_label = self.ax.text(0, 0, '', color='red', fontweight='bold', fontsize=10, zorder=11)
        self.ax.legend(loc='upper right')

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

    def pos3_raw_calc(self, distances):
        if len(distances) < 3: return None, None
        A, b = [], []
        ids = ['A1', 'A2', 'A3']
        for i in range(2):
            p1, p2 = self.anchor_positions[ids[i]], self.anchor_positions[ids[i+1]]
            r1, r2 = distances[ids[i]], distances[ids[i+1]]
            A.append([2*(p2[0]-p1[0]), 2*(p2[1]-p1[1])])
            b.append(r1**2 - r2**2 - p1[0]**2 - p1[1]**2 + p2[0]**2 + p2[1]**2)
        try: return np.linalg.solve(A, b)
        except: return None, None

    def update_plot(self, frame):
        latest_data = None
        while True:
            try:
                data, _ = self.sock.recvfrom(2048)
                latest_data = data
            except: break

        if latest_data:
            try:
                jd = json.loads(latest_data.decode())
                dists = {a['id']: max(0.01, a['distance']) for a in jd['anchors']}
                
                self.ekf.predict()
                self.ekf.update(dists, self.anchor_positions)
                sx, sy, speed = self.ekf.get_state()

                # 邊界鎖定
                sx = max(self.limit_x[0], min(self.limit_x[1], sx))
                sy = max(self.limit_y[0], min(self.limit_y[1], sy))

                # 更新圓圈
                for aid, circle in self.circles.items():
                    if aid in dists:
                        circle.set_radius(dists[aid])
                        circle.set_visible(self.show_circles)
                    else:
                        circle.set_visible(False)

                # 更新 Raw Point
                rx, ry = self.pos3_raw_calc(dists)
                if rx is not None and self.show_raw:
                    self.raw_point.set_data([rx], [ry])
                else:
                    self.raw_point.set_data([], [])

                # 更新 EKF Tag
                if self.show_ekf:
                    self.tag_point.set_data([sx], [sy])
                    self.tag_label.set_position((sx + 0.2, sy + 0.2))
                    self.tag_label.set_text(f"Pos:({sx:.2f}, {sy:.2f})\nSpd:{speed:.2f} m/s")
                    self.trail_x.append(sx); self.trail_y.append(sy)
                    if len(self.trail_x) > 40: self.trail_x.pop(0); self.trail_y.pop(0)
                    self.tag_trail.set_data(self.trail_x, self.trail_y)
                else:
                    self.tag_point.set_data([], [])
                    self.tag_trail.set_data([], [])
                    self.tag_label.set_text("")

            except Exception as e:
                print(f"Data Error: {e}")

        # 重要：回傳清單必須包含所有需要被「畫」出來的東西，包括 Anchor
        return (self.tag_point, self.raw_point, self.tag_label, self.tag_trail, 
                *self.circles.values(), *self.anchor_plots)

    def run(self):
        # 降低 interval 到 20ms 以維持穩定性
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=20, blit=True, cache_frame_data=False)
        plt.show()

if __name__ == "__main__":
    UWBPositionSystem().run()