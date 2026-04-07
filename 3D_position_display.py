import time
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

            # 預測距離
            z_pred = np.sqrt(dx**2 + dy**2 + dz**2)
            if z_pred < 0.01: z_pred = 0.01
            
            H = np.array([[dx/z_pred, dy/z_pred, dz/z_pred, 0, 0, 0]])
            
            # 計算殘差
            innovation = z_meas - z_pred
        
            S = np.dot(np.dot(H, self.P), H.T) + self.R_val
            S_val = S[0,0]
            
            # 異常值剔除
            mahalanobis_sq = (innovation**2) / S_val
            if mahalanobis_sq > 9.0: continue
            
            
            # S = np.dot(np.dot(H, self.P), H.T) + self.R_val
            K = np.dot(self.P, H.T) / S_val
            self.X += K * innovation
            self.P = np.dot((np.eye(6) - np.dot(K, H)), self.P)

    def get_pos(self):
        return self.X[0, 0], self.X[1, 0], self.X[2, 0]

class MultiTagSystem:
    def __init__(self):
        self.target_n = 1000
        self.status = '加密47pad'
        
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
        self.tags = {'T1': '#4CAF50', 'T2': '#2E7D32', 'T3': '#E91E63'}
        self.ekfs = {tid: UWB_EKF_3D() for tid in self.tags}
        self.trails = {tid: ([], [], []) for tid in self.tags}
        self.raw_history = {tid: [] for tid in self.tags}
        self.history_data = {tid: {'residual': [], 'jump': []} for tid in self.tags}
        self.has_saved = {tid: False for tid in self.tags}
        
        # 創建畫布
        self.fig = plt.figure(figsize=(15, 7))
        self.ax2d = self.fig.add_subplot(121)
        self.ax3d = self.fig.add_subplot(122, projection='3d')

        self.fig.canvas.mpl_connect('close_event', self.on_close)
        
        self.setup_axes()

    def setup_axes(self):
        # anchor 位置
        all_x = [p[0] for p in self.anchors.values()]
        all_y = [p[1] for p in self.anchors.values()]
        all_z = [p[2] for p in self.anchors.values()]
        pad = 1
        
        # 2D 設置
        self.ax2d.set_title("2D Position Tracking")
        #self.ax2d.set_xlim([-1, 9]); self.ax2d.set_ylim([-1, 7])
        self.ax2d.set_xlim([min(all_x)-pad, max(all_x)+pad])
        self.ax2d.set_ylim([min(all_y)-pad, max(all_y)+pad])
        self.ax2d.grid(True); self.ax2d.set_aspect('equal')
        
        # 3D 設置
        self.ax3d.set_title("3D Position Tracking")
        #self.ax3d.set_xlim([-1, 9]); self.ax3d.set_ylim([-1, 7]); self.ax3d.set_zlim([0, 4])
        self.ax3d.set_xlim([min(all_x)-pad, max(all_x)+pad])
        self.ax3d.set_ylim([min(all_y)-pad, max(all_y)+pad])
        self.ax3d.set_zlim([min(all_z)-pad, max(all_z)+pad])
        
        # 畫出 Anchor
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
            cloud2d, = self.ax2d.plot([], [], 'kx', markersize=4, alpha=0.2)
            status2d = self.ax2d.text(0, 0, '', fontsize=8, color='black', bbox=dict(facecolor='white', alpha=0.5))
            
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
                'pt2d': pt2d, 'ln2d': ln2d, 'cloud2d': cloud2d, 'status2d': status2d,
                'pt3d': pt3d, 'ln3d': ln3d, 'raw2d': raw2d, 'raw3d': raw3d
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
                ekf.predict() # 預測
                
                # 原始定位
                raw_xyz = self.trilateration_3d(dists)
                
                # 更新
                ekf.update(dists, self.anchors)
                
                # 記錄軌跡
                x, y, z = ekf.get_pos()
                tx, ty, tz = self.trails[tid]
                tx.append(x); ty.append(y); tz.append(z)
                if len(tx) > 30: tx.pop(0); ty.pop(0); tz.pop(0)
                
                updated_tags.add(tid)
                
                # Raw Data 顯示
                if raw_xyz is not None:
                    # 挑變速率
                    raw_vel = 0
                    if self.raw_history[tid]:
                        prev_raw = self.raw_history[tid][-1]
                        raw_vel = np.linalg.norm(raw_xyz - prev_raw)

                    # 剩餘誤差
                    residual = self.calculate_residual(raw_xyz, dists)
                    
                    # 軌跡雲
                    self.history_data[tid]['residual'].append(residual)
                    self.history_data[tid]['jump'].append(raw_vel)
                    self.raw_history[tid].append(raw_xyz)
                    
                    ## 更新畫面
                    objs = self.plot_objs[tid]
                    objs['raw2d'].set_data([raw_xyz[0]], [raw_xyz[1]])
                    objs['raw3d'].set_data_3d([raw_xyz[0]], [raw_xyz[1]], [raw_xyz[2]])
                    N = len(np.array(self.history_data['T1']['residual']))
                    print(N)
                    
                    # 軌跡雲
                    raw_pts = np.array(self.raw_history[tid])
                    objs['cloud2d'].set_data(raw_pts[:, 0], raw_pts[:, 1])
                    
                    # 文字資訊
                    '''
                    info_text = (f"Tag: {tid}\n"
                                 f"Residual: {residual*100:.1f}cm\n"
                                 f"Raw Jump: {raw_vel*100:.1f}cm")
                    objs['status2d'].set_position((raw_xyz[0] + 0.1, raw_xyz[1] + 0.1))
                    objs['status2d'].set_text(info_text)'''
                    
                    # 存檔
                    for tid in self.tags:
                        current_n = len(self.history_data[tid]['residual'])
                        if current_n >= self.target_n and not self.has_saved[tid]:
                            self.save_all_plots(tid, current_n)
                            self.has_saved[tid] = True
                    
            except BlockingIOError: break
            except Exception:       break

        # 刷新畫布上受影響的對象
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
        self.plot_statistics()
    
    def plot_statistics(self):
        for tid in self.tags:
            res_data = np.array(self.history_data[tid]['residual'])
            jump_data = np.array(self.history_data[tid]['jump'])
            
            if len(res_data) == 0: continue
            
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
            
            # 左圖：Residual (剩餘誤差)
            avg_res = np.mean(res_data)
            std_res = np.std(res_data)
            ax1.plot(res_data, color='blue', marker='o', markersize=3, alpha=0.5, linestyle='-', linewidth=0.5)
            ax1.axhline(avg_res, color='red', linestyle='--', label=f'Avg: {avg_res:.4f} m')
            ax1.set_title(f"Residual (Error) - Tag {tid}\nAvg={avg_res:.4f} m | Std={std_res:.4f} m | N={len(res_data)}")
            ax1.set_ylabel("Error (m)")
            ax1.set_xlabel("Sample Index")
            ax1.grid(True, alpha=0.3)
            ax1.legend()

            # 右圖：Raw Jump (跳動速率/位移)
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
    
    def save_all_plots(self, tid, n_count):
        """2D、3D定位"""
        self.fig.savefig(f"{self.status}_{tid}_N{self.target_n}_{time.strftime("%Y%m%d_%H%M%S")}-1.png")
        
        """存檔剩餘誤差、跳動速率"""
        res_data = np.array(self.history_data[tid]['residual'])
        jump_data = np.array(self.history_data[tid]['jump'])
        
        # 只取前 target_n 筆數據，確保存出來的圖正是你設定的長度
        res_plot = res_data[:self.target_n]
        jump_plot = jump_data[:self.target_n]

        fig_save, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
        
        # 左圖：Residual (剩餘誤差)
        avg_res = np.mean(res_data)
        std_res = np.std(res_data)
        ax1.plot(res_data, color='blue', marker='o', markersize=3, alpha=0.5, linestyle='-', linewidth=0.5)
        ax1.axhline(avg_res, color='red', linestyle='--', label=f'Avg: {avg_res:.4f} m')
        ax1.set_title(f"Residual (Error) - Tag {tid}\nAvg={avg_res:.4f} m | Std={std_res:.4f} m | N={len(res_data)}")
        ax1.set_ylabel("Error (m)")
        ax1.set_xlabel("Sample Index")
        ax1.grid(True, alpha=0.3)
        ax1.legend()

        # 右圖：Raw Jump (跳動速率/位移)
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
        fig_save.savefig(f"{self.status}_{tid}_N{self.target_n}_{time.strftime("%Y%m%d_%H%M%S")}-2.png")
        plt.close(fig_save) 
    
    def run(self):
        self.ani = FuncAnimation(self.fig, self.update, interval=20, blit=False)
        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    MultiTagSystem().run()