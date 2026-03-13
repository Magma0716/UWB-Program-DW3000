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
        self.Q = np.eye(6) * 0.005 
        self.R_val = 0.1 

    def predict(self):
        self.X = np.dot(self.A, self.X)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q

    def update(self, distances, anchor_pos):
        for aid, z_meas in distances.items():
            if aid not in anchor_pos: continue
            an_x, an_y, an_z = anchor_pos[aid]
            dx, dy, dz = self.X[0,0]-an_x, self.X[1,0]-an_y, self.X[2,0]-an_z
            z_pred = np.sqrt(dx**2 + dy**2 + dz**2)
            if z_pred < 0.001: z_pred = 0.001
            H = np.array([[dx/z_pred, dy/z_pred, dz/z_pred, 0, 0, 0]])
            S = np.dot(np.dot(H, self.P), H.T) + self.R_val
            K = np.dot(np.dot(self.P, H.T), np.linalg.inv(S))
            self.X += np.dot(K, (z_meas - z_pred))
            self.P = np.dot((np.eye(6) - np.dot(K, H)), self.P)

    def get_pos(self):
        return self.X[0, 0], self.X[1, 0], self.X[2, 0]

class MultiTagSystem:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('192.168.0.108', 8001))
        self.sock.setblocking(False)
        
        # 設置 4 個 Anchor 位置
        self.anchors = {
            'A1': (0.0, 0.0, 0.0),
            'A2': (2.0, 0.0, 0.0),
            'A3': (1.0, 1.732, 0.0),
            'A4': (1.0, 0.577, 1.633),
        }
        
        # 初始化多個 Tag 的 EKF 與 軌跡
        self.tags = {'T1': '#4CAF50', 'T2': '#2E7D32', 'T3': '#E91E63'} # ID: 顏色
        self.ekfs = {tid: UWB_EKF_3D() for tid in self.tags}
        self.trails = {tid: ([], [], []) for tid in self.tags}

        # 創建畫布：1 列 2 欄 (左 2D, 右 3D)
        self.fig = plt.figure(figsize=(15, 7))
        self.ax2d = self.fig.add_subplot(121)
        self.ax3d = self.fig.add_subplot(122, projection='3d')

        self.setup_axes()

    def setup_axes(self):
        # 2D 設置
        self.ax2d.set_title("2D Position Tracking")
        self.ax2d.set_xlim([-1, 9]); self.ax2d.set_ylim([-1, 7])
        self.ax2d.grid(True); self.ax2d.set_aspect('equal')
        
        # 3D 設置
        self.ax3d.set_title("3D Position Tracking")
        self.ax3d.set_xlim([-1, 9]); self.ax3d.set_ylim([-1, 7]); self.ax3d.set_zlim([0, 4])
        
        # 畫出 Anchor (藍色方塊)
        for aid, pos in self.anchors.items():
            self.ax2d.plot(pos[0], pos[1], 'bs', markersize=8, label=aid if 'A1' in aid else "")
            self.ax3d.scatter(pos[0], pos[1], pos[2], color='blue', marker='s', s=60)

        # 初始化繪圖對象
        self.plot_objs = {}
        for tid, color in self.tags.items():
            ## EKF
            # 2D
            ln2d, = self.ax2d.plot([], [], color=color, alpha=0.3)
            pt2d, = self.ax2d.plot([], [], color=color, marker='o', markersize=10, label=f'Tag {tid}')
            
            # 3D
            ln3d, = self.ax3d.plot([], [], [], color=color, alpha=0.3)
            pt3d, = self.ax3d.plot([], [], [], color=color, marker='o', markersize=8)
            
            ## raw data
            # 2D
            raw2d, = self.ax2d.plot([], [], 'kx', markersize=8, markeredgewidth=1, label='Raw Data' if tid == 'T1' else "")
            # 3D
            raw3d, = self.ax3d.plot([], [], [], 'kx', markersize=8, markeredgewidth=1, label='Raw Data' if tid == 'T1' else "")
            
            # for update 
            self.plot_objs[tid] = {
                'pt2d': pt2d, 'ln2d': ln2d, 
                'pt3d': pt3d, 'ln3d': ln3d,
                'raw2d': raw2d, 'raw3d': raw3d
            }
            
        #self.ax2d.legend(loc='upper right', fontsize='small')

    def trilateration_3d(self, dists):
        """
        dists: {'A1': d1, 'A2': d2, ...}
        返回: (x, y, z) 的原始定位點
        """
        if len(dists) < 4:
            return None
            
        p1 = np.array(self.anchors['A1'])
        d1 = dists['A1']
        
        A, B = [], []
        for aid in ['A2', 'A3', 'A4']:
            pi = np.array(self.anchors[aid])
            di = dists[aid]
            # 線性化矩陣：2(Pi - P1) * X = d1^2 - di^2 - |P1|^2 + |Pi|^2
            A.append(2 * (pi - p1))
            B.append(d1**2 - di**2 - np.sum(p1**2) + np.sum(pi**2))
            
        try:
            # 最小平方法求解 (AX = B)
            raw_pos, _, _, _ = np.linalg.lstsq(np.array(A), np.array(B), rcond=None)
            return raw_pos  # [x, y, z]
        except:
            return None
    
    def update(self, frame):
        latest_data = None
        while True:
            try:
                data, _ = self.sock.recvfrom(2048)
                latest_data = data
            except: break

        if latest_data:
            try:
                msg = json.loads(latest_data.decode())
                tid = msg.get('tag', 'T1')
                dists = {a['id']: a['distance'] for a in msg['anchors']}

                print(msg)
                
                # EKF data
                if tid in self.ekfs:
                    ekf = self.ekfs[tid]
                    ekf.predict()
                    ekf.update(dists, self.anchors)
                    x, y, z = ekf.get_pos()

                    # 更新軌跡數據
                    tx, ty, tz = self.trails[tid]
                    tx.append(x); ty.append(y); tz.append(z)
                    if len(tx) > 30: tx.pop(0); ty.pop(0); tz.pop(0)

                    # 更新圖形
                    objs = self.plot_objs[tid]
                    objs['pt2d'].set_data([x], [y])
                    objs['ln2d'].set_data(tx, ty)
                    objs['pt3d'].set_data_3d([x], [y], [z])
                    objs['ln3d'].set_data_3d(tx, ty, tz)
                
                # raw data
                raw_xyz = self.trilateration_3d(dists)
                if raw_xyz is not None:
                    rx, ry, rz = raw_xyz
                    self.plot_objs[tid]['raw2d'].set_data([rx], [ry])
                    self.plot_objs[tid]['raw3d'].set_data_3d([rx], [ry], [rz])
                else:
                    self.plot_objs[tid]['raw2d'].set_data([], [])
                    self.plot_objs[tid]['raw3d'].set_data_3d([], [], [])
                    
            except Exception as e: 
                pass

        return [] # FuncAnimation blit=False 不需要回傳

    def run(self):
        self.ani = FuncAnimation(self.fig, self.update, interval=20, blit=False)
        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    MultiTagSystem().run()