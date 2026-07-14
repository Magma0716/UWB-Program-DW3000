'''
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
'''

import socket
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# 3D EKF
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
            
            mahalanobis_sq = (innovation**2) / S_val
            if mahalanobis_sq > 9.0: continue
            
            K = np.dot(self.P, H.T) / S_val
            self.X += K * innovation
            self.P = np.dot((np.eye(6) - np.dot(K, H)), self.P)

    def get_pos(self):
        return self.X[0, 0], self.X[1, 0], self.X[2, 0]

class MultiTagSystem:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('192.168.0.108', 8001))
        self.sock.setblocking(False)
        
        self.anchors = {
            'A1': (0.0, 0.0, 0.0),
            'A2': (2.0, 0.0, 0.0),
            'A3': (1.0, 1.732, 0.0),
            'A4': (1.0, 0.577, 1.633),
        }
        
        self.tags = {'T1': '#4CAF50', 'T2': '#2E7D32', 'T3': '#E91E63'}
        self.ekfs = {tid: UWB_EKF_3D() for tid in self.tags}
        self.trails = {tid: ([], [], []) for tid in self.tags}
        self.raw_history = {tid: [] for tid in self.tags}

        # --- 新增：用於儲存統計數據的清單 ---
        self.history_data = {tid: {'residual': [], 'jump': []} for tid in self.tags}
        
        self.fig = plt.figure(figsize=(15, 7))
        self.ax2d = self.fig.add_subplot(121)
        self.ax3d = self.fig.add_subplot(122, projection='3d')

        # 註冊關閉視窗事件，以便在關閉後繪圖
        self.fig.canvas.mpl_connect('close_event', self.on_close)

        self.setup_axes()

    def setup_axes(self):
        all_x = [p[0] for p in self.anchors.values()]
        all_y = [p[1] for p in self.anchors.values()]
        all_z = [p[2] for p in self.anchors.values()]
        pad = 0.5
        
        self.ax2d.set_title("2D Position Tracking")
        self.ax2d.set_xlim([min(all_x)-pad, max(all_x)+pad])
        self.ax2d.set_ylim([min(all_y)-pad, max(all_y)+pad])
        self.ax2d.grid(True, linestyle=':', alpha=0.6)
        self.ax2d.set_aspect('equal')
        
        self.ax3d.set_title("3D Position Tracking")
        self.ax3d.set_xlim([min(all_x)-pad, max(all_x)+pad])
        self.ax3d.set_ylim([min(all_y)-pad, max(all_y)+pad])
        self.ax3d.set_zlim([min(all_z)-pad, max(all_z)+pad])
        
        for aid, pos in self.anchors.items():
            self.ax2d.plot(pos[0], pos[1], 'bs', markersize=8, label=aid if 'A1' in aid else "")
            self.ax3d.scatter(pos[0], pos[1], pos[2], color='blue', marker='s', s=60)

        self.plot_objs = {}
        for tid, color in self.tags.items():
            ln2d, = self.ax2d.plot([], [], color=color, alpha=0.3)
            pt2d, = self.ax2d.plot([], [], color=color, marker='o', markersize=10, label=f'Tag {tid}')
            cloud2d, = self.ax2d.plot([], [], 'kx', markersize=2, alpha=0.1)
            status2d = self.ax2d.text(0, 0, '', fontsize=8, color='black', bbox=dict(facecolor='white', alpha=0.5))
            
            ln3d, = self.ax3d.plot([], [], [], color=color, alpha=0.3)
            pt3d, = self.ax3d.plot([], [], [], color=color, marker='o', markersize=8)
            raw2d, = self.ax2d.plot([], [], 'kx', markersize=8, markeredgewidth=1)
            raw3d, = self.ax3d.plot([], [], [], 'kx', markersize=8, markeredgewidth=1)
            
            self.plot_objs[tid] = {
                'pt2d': pt2d, 'ln2d': ln2d, 'cloud2d': cloud2d, 'status2d': status2d,
                'pt3d': pt3d, 'ln3d': ln3d, 'raw2d': raw2d, 'raw3d': raw3d
            }

    def trilateration_3d(self, dists):
        if len(dists) < 4: return None
        p1 = np.array(self.anchors['A1'])
        d1 = dists['A1']
        A, B = [], []
        for aid in ['A2', 'A3', 'A4']:
            pi = np.array(self.anchors[aid])
            di = dists[aid]
            A.append(2 * (pi - p1))
            B.append(d1**2 - di**2 - np.sum(p1**2) + np.sum(pi**2))
        try:
            raw_pos, _, _, _ = np.linalg.lstsq(np.array(A), np.array(B), rcond=None)
            return raw_pos
        except: return None
    
    def calculate_residual(self, pos, dists):
        errors = []
        for aid, d_meas in dists.items():
            if aid in self.anchors:
                anc_pos = np.array(self.anchors[aid])
                d_calc = np.linalg.norm(pos - anc_pos)
                errors.append((d_calc - d_meas)**2)
        return np.sqrt(np.mean(errors)) if errors else 0
    
    def update(self, frame):
        updated_tags = set()
        while True:
            try:
                data, _ = self.sock.recvfrom(2048)
                msg = json.loads(data.decode())
                tid = msg.get('tag')
                if tid not in self.ekfs: continue
                
                dists = {a['id']: a['distance'] for a in msg['anchors']}
                ekf = self.ekfs[tid]
                ekf.predict()
                
                raw_xyz = self.trilateration_3d(dists)
                ekf.update(dists, self.anchors)
                
                x, y, z = ekf.get_pos()
                tx, ty, tz = self.trails[tid]
                tx.append(x); ty.append(y); tz.append(z)
                if len(tx) > 30: tx.pop(0); ty.pop(0); tz.pop(0)
                
                updated_tags.add(tid)
                
                if raw_xyz is not None:
                    raw_vel = 0
                    if self.raw_history[tid]:
                        prev_raw = self.raw_history[tid][-1]
                        raw_vel = np.linalg.norm(raw_xyz - prev_raw)

                    residual = self.calculate_residual(raw_xyz, dists)
                    
                    # --- 紀錄數據 ---
                    self.history_data[tid]['residual'].append(residual)
                    self.history_data[tid]['jump'].append(raw_vel)
                    self.raw_history[tid].append(raw_xyz)
                    
                    objs = self.plot_objs[tid]
                    objs['raw2d'].set_data([raw_xyz[0]], [raw_xyz[1]])
                    objs['raw3d'].set_data_3d([raw_xyz[0]], [raw_xyz[1]], [raw_xyz[2]])
                    raw_pts = np.array(self.raw_history[tid])
                    objs['cloud2d'].set_data(raw_pts[:, 0], raw_pts[:, 1])
                    
                    info_text = (f"Tag: {tid}\nRes: {residual*100:.1f}cm\nJump: {raw_vel*100:.1f}cm")
                    objs['status2d'].set_position((raw_xyz[0] + 0.1, raw_xyz[1] + 0.1))
                    objs['status2d'].set_text(info_text)
                    
            except (BlockingIOError, Exception): break

        for tid in updated_tags:
            x, y, z = self.ekfs[tid].get_pos()
            tx, ty, tz = self.trails[tid]
            objs = self.plot_objs[tid]
            objs['pt2d'].set_data([x], [y])
            objs['ln2d'].set_data(tx, ty)
            objs['pt3d'].set_data_3d([x], [y], [z])
            objs['ln3d'].set_data_3d(tx, ty, tz)
        return []

    def on_close(self, event):
        """當視窗關閉時觸發繪圖"""
        print("Closing Tracking System... Generating statistics plots.")
        self.plot_statistics()

    def plot_statistics(self):
        """繪製類似上傳圖片風格的分析圖"""
        for tid in self.tags:
            res_data = np.array(self.history_data[tid]['residual'])
            jump_data = np.array(self.history_data[tid]['jump'])
            
            if len(res_data) == 0: continue

            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
            
            # --- 左圖：Residual (剩餘誤差) ---
            avg_res = np.mean(res_data)
            std_res = np.std(res_data)
            ax1.plot(res_data, color='blue', marker='o', markersize=3, alpha=0.5, linestyle='-', linewidth=0.5)
            ax1.axhline(avg_res, color='red', linestyle='--', label=f'Avg: {avg_res:.4f} m')
            ax1.set_title(f"Residual (Error) - Tag {tid}\nAvg={avg_res:.4f} m | Std={std_res:.4f} m | N={len(res_data)}")
            ax1.set_ylabel("Error (m)")
            ax1.set_xlabel("Sample Index")
            ax1.grid(True, alpha=0.3)
            ax1.legend()

            # --- 右圖：Raw Jump (跳動速率/位移) ---
            avg_jump = np.mean(jump_data)
            std_jump = np.std(jump_data)
            ax2.plot(jump_data, color='orange', marker='x', markersize=3, alpha=0.6, linestyle='-', linewidth=0.5)
            ax2.axhline(avg_jump, color='red', linestyle='--', label=f'Avg: {avg_jump:.4f} m')
            ax2.set_title(f"Raw Data Jump - Tag {tid}\nAvg={avg_jump:.4f} m | Std={std_jump:.4f} m | N={len(jump_data)}")
            ax2.set_ylabel("Displacement (m)")
            ax2.set_xlabel("Sample Index")
            ax2.grid(True, alpha=0.3)
            ax2.legend()

            plt.tight_layout()
        
        plt.show()

    def run(self):
        self.ani = FuncAnimation(self.fig, self.update, interval=20, blit=False)
        plt.show()

if __name__ == "__main__":
    MultiTagSystem().run()