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

    # --- 【新增】取得當前定位的不確定性 (公尺) ---
    def get_uncertainty(self):
        # 取出 P 矩陣中對應 X 和 Y 座標的方差並開根號 (標準差)
        return np.sqrt(self.P[0, 0]), np.sqrt(self.P[1, 1])

class MultiTagSystem:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 請確保此 IP 與你電腦 IP 一致
        self.sock.bind(('192.168.0.108', 8001))
        self.sock.setblocking(False)
        
        self.anchors = {
            'A1': (0.0, 0.0, 0.0), 'A2': (2.0, 0.0, 0.0),
            'A3': (1.0, 1.732, 0.0), 'A4': (1.0, 0.577, 1.633),
        }
        
        self.tags = {'T1': '#4CAF50', 'T2': '#2E7D32', 'T3': '#E91E63'}
        self.ekfs = {tid: UWB_EKF_3D() for tid in self.tags}
        self.trails = {tid: ([], [], []) for tid in self.tags}

        self.fig = plt.figure(figsize=(15, 7))
        self.ax2d = self.fig.add_subplot(121)
        self.ax3d = self.fig.add_subplot(122, projection='3d')

        self.setup_axes()

    def setup_axes(self):
        all_x = [p[0] for p in self.anchors.values()]; all_y = [p[1] for p in self.anchors.values()]; all_z = [p[2] for p in self.anchors.values()]
        pad = 0.5
        self.ax2d.set_title("2D Tracking (Circle = Uncertainty)")
        self.ax2d.set_xlim([min(all_x)-pad, max(all_x)+pad]); self.ax2d.set_ylim([min(all_y)-pad, max(all_y)+pad])
        self.ax2d.grid(True); self.ax2d.set_aspect('equal')
        
        self.ax3d.set_title("3D Position Tracking")
        self.ax3d.set_xlim([min(all_x)-pad, max(all_x)+pad]); self.ax3d.set_ylim([min(all_y)-pad, max(all_y)+pad]); self.ax3d.set_zlim([min(all_z)-pad, max(all_z)+pad])
        
        for aid, pos in self.anchors.items():
            self.ax2d.plot(pos[0], pos[1], 'bs', markersize=8)
            self.ax3d.scatter(pos[0], pos[1], pos[2], color='blue', marker='s', s=60)

        self.plot_objs = {}
        for tid, color in self.tags.items():
            ln2d, = self.ax2d.plot([], [], color=color, alpha=0.3)
            pt2d, = self.ax2d.plot([], [], color=color, marker='o', markersize=10, label=f'Tag {tid}')
            
            # --- 【新增】不確定性圓圈 (信心半徑) ---
            circle = plt.Circle((0, 0), 0.1, color=color, fill=True, alpha=0.1)
            self.ax2d.add_patch(circle)
            
            # --- 【新增】顯示文字資訊 ---
            txt2d = self.ax2d.text(0, 0, '', fontsize=9, color=color)

            ln3d, = self.ax3d.plot([], [], [], color=color, alpha=0.3)
            pt3d, = self.ax3d.plot([], [], [], color=color, marker='o', markersize=8)
            raw2d, = self.ax2d.plot([], [], 'kx', markersize=8, markeredgewidth=1)
            raw3d, = self.ax3d.plot([], [], [], 'kx', markersize=8, markeredgewidth=1)
            
            self.plot_objs[tid] = {
                'pt2d': pt2d, 'ln2d': ln2d, 'circle': circle, 'txt2d': txt2d,
                'pt3d': pt3d, 'ln3d': ln3d, 'raw2d': raw2d, 'raw3d': raw3d
            }

    def trilateration_3d(self, dists):
        if len(dists) < 4: return None
        p1 = np.array(self.anchors['A1']); d1 = dists['A1']
        A, B = [], []
        for aid in ['A2', 'A3', 'A4']:
            pi = np.array(self.anchors[aid]); di = dists[aid]
            A.append(2 * (pi - p1))
            B.append(d1**2 - di**2 - np.sum(p1**2) + np.sum(pi**2))
        try:
            raw_pos, _, _, _ = np.linalg.lstsq(np.array(A), np.array(B), rcond=None)
            return raw_pos
        except: return None
    
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
                
                # --- 【新增】計算偏移殘差 ---
                offset = 0
                if raw_xyz is not None:
                    offset = np.linalg.norm(np.array([x,y,z]) - raw_xyz)
                    self.plot_objs[tid]['raw2d'].set_data([raw_xyz[0]], [raw_xyz[1]])
                    self.plot_objs[tid]['raw3d'].set_data_3d([raw_xyz[0]], [raw_xyz[1]], [raw_xyz[2]])
                
                # 更新圓圈與文字
                sigma_x, sigma_y = ekf.get_uncertainty()
                objs = self.plot_objs[tid]
                objs['circle'].set_center((x, y))
                objs['circle'].set_radius(sigma_x + sigma_y) # 半徑反映不確定性
                objs['txt2d'].set_position((x + 0.1, y + 0.1))
                objs['txt2d'].set_text(f'Off: {offset*100:.1f}cm') # 顯示公分誤差

                updated_tags.add(tid)

            except BlockingIOError: break
            except Exception as e: break

        for tid in updated_tags:
            x, y, z = self.ekfs[tid].get_pos()
            tx, ty, tz = self.trails[tid]
            objs = self.plot_objs[tid]
            objs['pt2d'].set_data([x], [y])
            objs['ln2d'].set_data(tx, ty)
            objs['pt3d'].set_data_3d([x], [y], [z])
            objs['ln3d'].set_data_3d(tx, ty, tz)

        return []

    def run(self):
        self.ani = FuncAnimation(self.fig, self.update, interval=20, blit=False)
        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    MultiTagSystem().run()