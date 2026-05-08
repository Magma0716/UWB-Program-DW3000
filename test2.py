import time
import socket
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import tkinter as tk
from tkinter import messagebox
from matplotlib.widgets import Button

# ================================
# ========== 系統全局配置區 ========
# ================================
CONFIG = {
    "ENABLE_STATS_EXPORT": True,    # 是否計算並輸出 Residual/Jump 圖表與存檔
    "SHOW_CLOUD_POINTS": True,      # 初始狀態：歷史路徑雲 ('x' 點)
    "SHOW_RAW_POINTS": True,        # 初始狀態：當前原始測量點 (Raw Data)
    "SHOW_PREDICT_POINTS": False,   # 初始狀態：EKF 預測後的點與連線
    "TARGET_SAMPLES": 1000,         # 達到多少樣本後自動存檔
}

class UWB_EKF_3D:
    def __init__(self, x0=0, y0=0, z0=0, dt=0.016):
        self.X = np.array([[x0], [y0], [z0], [0], [0], [0]], dtype=float)
        self.dt = dt
        self.A = np.eye(6)
        self.A[0, 3] = self.A[1, 4] = self.A[2, 5] = self.dt
        self.P = np.eye(6) * 1.0
        self.Q = np.eye(6) * 0.01
        self.R_val = 0.5

    def predict(self):
        self.X = np.dot(self.A, self.X)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q

    def update(self, distances, anchor_pos):
        for aid, z_meas in distances.items():
            if aid not in anchor_pos: continue
            an_x, an_y, an_z = anchor_pos[aid]
            dx, dy, dz = self.X[0,0]-an_x, self.X[1,0]-an_y, self.X[2,0]-an_z
            z_pred = np.sqrt(dx**2 + dy**2 + dz**2)
            if z_pred < 0.01: z_pred = 0.01
            H = np.array([[dx/z_pred, dy/z_pred, dz/z_pred, 0, 0, 0]])
            innovation = z_meas - z_pred
            S = np.dot(np.dot(H, self.P), H.T) + self.R_val
            S_val = S[0,0]
            if (innovation**2) / S_val > 9.0: continue
            K = np.dot(self.P, H.T) / S_val
            self.X += K * innovation
            self.P = np.dot((np.eye(6) - np.dot(K, H)), self.P)

    def get_pos(self):
        return self.X[0, 0], self.X[1, 0], self.X[2, 0]

class MultiTagSystem:
    def __init__(self):
        self.target_n = CONFIG["TARGET_SAMPLES"]
        self.status = 'AES加密'
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('192.168.0.108', 8001))
        self.sock.setblocking(False)
        
        self.root = tk.Tk(); self.root.withdraw()
        
        self.anchors = {'A1': (0.0, 0.0, 0.0), 'A2': (2.0, 0.0, 0.0), 'A3': (1.0, 1.732, 0.0), 'A4': (1.0, 0.0, 0.4)} #'A4': (1.0, 0.577, 1.633)
        self.tags = {'T1': '#4CAF50', 'T2': "#3BD1DB", 'T3': '#E91E63', 'T4': "#BE5DFF"}
        self.ekfs = {tid: UWB_EKF_3D() for tid in self.tags}
        self.trails = {tid: ([], [], []) for tid in self.tags}
        self.raw_history = {tid: [] for tid in self.tags}
        self.history_data = {tid: {'residual': [], 'jump': [], 'tof': []} for tid in self.tags}
        self.has_saved = {tid: False for tid in self.tags}
        
        # 控制變數
        self.is_rotating = False
        self.angle = 0
        self.show_cloud = CONFIG["SHOW_CLOUD_POINTS"]
        self.show_raw = CONFIG["SHOW_RAW_POINTS"]
        self.show_predict = CONFIG["SHOW_PREDICT_POINTS"]
        
        # 圖表初始化 (增加底部留白空間給按鈕)
        self.fig = plt.figure(figsize=(15, 8.5))
        plt.subplots_adjust(bottom=0.15) 
        self.ax2d = self.fig.add_subplot(121)
        self.ax3d = self.fig.add_subplot(122, projection='3d')
        
        # --- 按鈕配置 ---
        self.ax_rotate = plt.axes([0.15, 0.02, 0.12, 0.045])
        self.btn_rotate = Button(self.ax_rotate, 'Rotate: OFF', color='#f0f0f0')
        self.btn_rotate.on_clicked(self.toggle_rotation)

        self.ax_cloud = plt.axes([0.30, 0.02, 0.12, 0.045])
        self.btn_cloud = Button(self.ax_cloud, f'Cloud: {"ON" if self.show_cloud else "OFF"}', color='#f0f0f0')
        self.btn_cloud.on_clicked(self.toggle_cloud)

        self.ax_raw = plt.axes([0.45, 0.02, 0.12, 0.045])
        self.btn_raw = Button(self.ax_raw, f'Raw: {"ON" if self.show_raw else "OFF"}', color='#f0f0f0')
        self.btn_raw.on_clicked(self.toggle_raw)

        self.ax_predict = plt.axes([0.60, 0.02, 0.12, 0.045])
        self.btn_predict = Button(self.ax_predict, f'EKF: {"ON" if self.show_predict else "OFF"}', color='#f0f0f0')
        self.btn_predict.on_clicked(self.toggle_predict)

        self.fig.canvas.mpl_connect('close_event', self.on_close)
        self.setup_axes()

    # --- 按鈕觸發函數 ---
    def toggle_rotation(self, event):
        self.is_rotating = not self.is_rotating
        self.btn_rotate.label.set_text(f"Rotate: {'ON' if self.is_rotating else 'OFF'}")

    def toggle_cloud(self, event):
        self.show_cloud = not self.show_cloud
        self.btn_cloud.label.set_text(f"Cloud: {'ON' if self.show_cloud else 'OFF'}")
        for tid in self.plot_objs:
            self.plot_objs[tid]['cloud2d'].set_visible(self.show_cloud)
            self.plot_objs[tid]['cloud3d'].set_visible(self.show_cloud)

    def toggle_raw(self, event):
        self.show_raw = not self.show_raw
        self.btn_raw.label.set_text(f"Raw: {'ON' if self.show_raw else 'OFF'}")
        for tid in self.plot_objs:
            self.plot_objs[tid]['raw2d'].set_visible(self.show_raw)
            self.plot_objs[tid]['raw3d'].set_visible(self.show_raw)

    def toggle_predict(self, event):
        self.show_predict = not self.show_predict
        self.btn_predict.label.set_text(f"EKF: {'ON' if self.show_predict else 'OFF'}")
        for tid in self.plot_objs:
            self.plot_objs[tid]['pt2d'].set_visible(self.show_predict)
            self.plot_objs[tid]['ln2d'].set_visible(self.show_predict)
            self.plot_objs[tid]['pt3d'].set_visible(self.show_predict)
            self.plot_objs[tid]['ln3d'].set_visible(self.show_predict)

    def setup_axes(self):
        all_x = [p[0] for p in self.anchors.values()]; all_y = [p[1] for p in self.anchors.values()]; all_z = [p[2] for p in self.anchors.values()]
        pad = 1
        self.ax2d.set_xlim([min(all_x)-pad, max(all_x)+pad]); self.ax2d.set_ylim([min(all_y)-pad, max(all_y)+pad])
        self.ax2d.grid(True); self.ax2d.set_aspect('equal')
        self.ax3d.set_xlim([min(all_x)-pad, max(all_x)+pad]); self.ax3d.set_ylim([min(all_y)-pad, max(all_y)+pad]); self.ax3d.set_zlim([min(all_z)-pad, max(all_z)+pad])
        
        for aid, pos in self.anchors.items():
            self.ax2d.plot(pos[0], pos[1], 'bs', markersize=8)
            self.ax3d.scatter(pos[0], pos[1], pos[2], color='blue', marker='s', s=60)

        self.plot_objs = {}
        for tid, color in self.tags.items():
            ln2d, = self.ax2d.plot([], [], color=color, alpha=0.3, linewidth=1.5, visible=self.show_predict)
            pt2d, = self.ax2d.plot([], [], color=color, marker='o', markersize=10, label=f'Tag {tid}', visible=self.show_predict)
            ln3d, = self.ax3d.plot([], [], [], color=color, alpha=0.3, visible=self.show_predict)
            pt3d, = self.ax3d.plot([], [], [], color=color, marker='o', markersize=8, visible=self.show_predict)
            
            cloud2d, = self.ax2d.plot([], [], 'x', color=color, markersize=4, alpha=0.2, visible=self.show_cloud)
            cloud3d, = self.ax3d.plot([], [], [], 'x', color=color, markersize=4, alpha=0.15, linestyle='None', visible=self.show_cloud)
            
            raw2d, = self.ax2d.plot([], [], 'x', color=color, markersize=8, markeredgewidth=1.5, visible=self.show_raw)
            raw3d, = self.ax3d.plot([], [], [], 'x', color=color, markersize=8, markeredgewidth=1.5, visible=self.show_raw)
            
            self.plot_objs[tid] = {'pt2d': pt2d, 'ln2d': ln2d, 'cloud2d': cloud2d, 'pt3d': pt3d, 'ln3d': ln3d, 'cloud3d': cloud3d, 'raw2d': raw2d, 'raw3d': raw3d}

    def trilateration_3d(self, dists):
        if len(dists) < 4: return None
        p1 = np.array(self.anchors['A1']); d1 = dists['A1']
        A, B = [], []
        for aid in ['A2', 'A3', 'A4']:
            pi, di = np.array(self.anchors[aid]), dists[aid]
            A.append(2 * (pi - p1)); B.append(d1**2 - di**2 - np.sum(p1**2) + np.sum(pi**2))
        try:
            raw_pos, _, _, _ = np.linalg.lstsq(np.array(A), np.array(B), rcond=None)
            return raw_pos
        except: return None

    def calculate_residual(self, pos, dists):
        errors = [(np.linalg.norm(pos - np.array(self.anchors[aid])) - d)**2 for aid, d in dists.items() if aid in self.anchors]
        return np.sqrt(np.mean(errors)) if errors else 0

    def update(self, frame):
        if self.is_rotating:
            self.angle = (self.angle + 1) % 360
            self.ax3d.view_init(elev=20, azim=self.angle)

        while True:
            try:
                data, _ = self.sock.recvfrom(2048)
                msg = json.loads(data.decode())
                tid = msg.get('tag')
                if tid not in self.ekfs: continue
                
                
                dists = {a['id']: a['distance'] for a in msg['anchors']}
                
                tof_list = [a['tof'] for a in msg['anchors'] if 'tof' in a]
                avg_tof = np.mean(tof_list) if tof_list else 0
                
                self.ekfs[tid].predict()
                raw_xyz = self.trilateration_3d(dists)
                self.ekfs[tid].update(dists, self.anchors)
                
                x, y, z = self.ekfs[tid].get_pos()
                tx, ty, tz = self.trails[tid]
                tx.append(x); ty.append(y); tz.append(z)
                if len(tx) > 30: tx.pop(0); ty.pop(0); tz.pop(0)
                
                if raw_xyz is not None:
                    if CONFIG["ENABLE_STATS_EXPORT"]:
                        raw_vel = np.linalg.norm(raw_xyz - self.raw_history[tid][-1]) if self.raw_history[tid] else 0
                        self.history_data[tid]['residual'].append(self.calculate_residual(raw_xyz, dists))
                        self.history_data[tid]['jump'].append(raw_vel)
                        self.history_data[tid]['tof'].append(avg_tof)
                    
                    self.raw_history[tid].append(raw_xyz)
                    objs = self.plot_objs[tid]
                    raw_pts = np.array(self.raw_history[tid])
                    
                    objs['raw2d'].set_data([raw_xyz[0]], [raw_xyz[1]])
                    objs['raw3d'].set_data_3d([raw_xyz[0]], [raw_xyz[1]], [raw_xyz[2]])
                    objs['cloud2d'].set_data(raw_pts[:, 0], raw_pts[:, 1])
                    objs['cloud3d'].set_data_3d(raw_pts[:, 0], raw_pts[:, 1], raw_pts[:, 2])
                    
                    # 更新 EKF 預測線
                    objs['pt2d'].set_data([x], [y])
                    objs['ln2d'].set_data(tx, ty)
                    objs['pt3d'].set_data_3d([x], [y], [z])
                    objs['ln3d'].set_data_3d(tx, ty, tz)

                    if CONFIG["ENABLE_STATS_EXPORT"] and len(self.history_data[tid]['residual']) >= self.target_n and not self.has_saved[tid]:
                        self.save_all_plots(tid); self.has_saved[tid] = True
                        messagebox.showinfo("存檔成功", f"Tag {tid} 數據已儲存。")
                                
            except BlockingIOError: break
            except Exception: break
        return []

    def save_all_plots(self, tid):
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.fig.savefig(f"{self.status}_{tid}_N{self.target_n}_{timestamp}_Track.png")
        
        res_plot = np.array(self.history_data[tid]['residual'][:self.target_n])
        jump_plot = np.array(self.history_data[tid]['jump'][:self.target_n])
        tof_plot = np.array(self.history_data[tid]['tof'][:self.target_n])
        
        fig_stat, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(16, 6))
        
        # 左圖：Residual (剩餘誤差)
        avg_res = np.mean(res_plot)
        std_res = np.std(res_plot)
        ax1.plot(res_plot, color='blue', marker='o', markersize=3, alpha=0.5, linestyle='-', linewidth=0.5)
        ax1.axhline(avg_res, color='red', linestyle='--', label=f'Avg: {avg_res:.4f} m')
        ax1.set_title(f"Residual (Error) - Tag {tid}\nAvg={avg_res:.4f} m | Std={std_res:.4f} m | N={len(res_plot)}")
        ax1.set_ylabel("Error (m)")
        ax1.set_xlabel("Sample Index")
        ax1.grid(True, alpha=0.3)
        ax1.legend()

        # 中圖：Raw Jump (跳動速率/位移)
        avg_jump = np.mean(jump_plot)
        std_jump = np.std(jump_plot)
        ax2.plot(jump_plot, color='green', marker='x', markersize=3, alpha=0.6, linestyle='-', linewidth=0.5)
        ax2.axhline(avg_jump, color='red', linestyle='--', label=f'Avg: {avg_jump:.4f} m')
        ax2.set_title(f"Raw Data Jump - Tag {tid}\nAvg={avg_jump:.4f} m | Std={std_jump:.4f} m | N={len(jump_plot)}")
        ax2.set_ylabel("Displacement (m)")
        ax2.set_xlabel("Sample Index")
        ax2.grid(True, alpha=0.3)
        ax2.legend()
        
        # 右圖：ms
        # right chart
        avg_tof = np.mean(tof_plot); std_tof = np.std(tof_plot)
        ax3.plot(tof_plot, color='orange', marker='x', markersize=3, alpha=0.8, linewidth=0.5)
        ax3.axhline(avg_tof, color='red', linestyle='--', label=f'Avg: {avg_tof:.1f} ns')
        ax3.set_title(f"Time of Flight Delay - Tag {tid}\nAvg={avg_tof:.2f} ns | Std={std_tof:.2f} ns | N={len(tof_plot)}")
        ax3.set_ylabel("TOF (ns)"); ax3.set_xlabel("Sample Index")
        ax3.grid(True, which='both', linestyle='-', alpha=0.2); ax3.legend(loc='upper right')
        
        plt.tight_layout(); 
        fig_stat.savefig(f"{self.status}_{tid}_N{self.target_n}_{timestamp}_Stats.png", dpi=300); 
        plt.close(fig_stat)

    def on_close(self, event): plt.show()
    def run(self):
        self.ani = FuncAnimation(self.fig, self.update, interval=20, blit=False)
        plt.show()

if __name__ == "__main__":
    MultiTagSystem().run()