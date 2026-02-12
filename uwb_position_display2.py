import socket
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from typing import Dict, Tuple, Optional


class UWBPositionSystem:
    def __init__(self):
        # ===============================
        # UDP 設定
        # ===============================
        self.bind_ip = "192.168.0.108"   # 你的電腦IP
        self.bind_port = 8001

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.bind_ip, self.bind_port))
        self.sock.settimeout(0.1)
        self.sock.setblocking(False)

        # ===============================
        # Anchor 位置（固定三點永遠畫出來）
        # ===============================
        self.anchor_positions = {
            "A1": (0.0, 0.0),
            "A2": (1.0, 0.0),
            #"A3": (4.0, 6.0),
        }

        self.max_trail_points = 50
        self.trail_x = []
        self.trail_y = []

        # ===============================
        # Matplotlib 初始化
        # ===============================
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.ax.set_xlim(-1, 9)
        self.ax.set_ylim(-1, 7)
        self.ax.grid(True)
        self.ax.set_title("UWB Position Tracking")
        self.ax.set_xlabel("X Position (meters)")
        self.ax.set_ylabel("Y Position (meters)")

        # Anchor 點（永遠顯示）
        self.anchor_artists = []
        for anchor_id, (x, y) in self.anchor_positions.items():
            artist, = self.ax.plot(x, y, "bs", markersize=10, label=anchor_id)
            self.anchor_artists.append(artist)

        # Tag 點 + 軌跡
        self.tag_point, = self.ax.plot([], [], "ro", markersize=10, label="Tag")
        self.tag_trail, = self.ax.plot([], [], "r-", alpha=0.5, label="Trail")

        # 距離文字（永遠存在，只是有收到才更新）
        self.distance_texts = {}
        for anchor_id, (x, y) in self.anchor_positions.items():
            text = self.ax.text(x, y + 0.2, f"{anchor_id}: --", fontsize=9)
            self.distance_texts[anchor_id] = text

        # 圖例（要在 Tag/Trail 建完後再叫）
        self.ax.legend(loc="upper right")

    # -------------------------------
    # 幾何：兩圓交點
    # -------------------------------
    def intersections(self, p1, r1, p2, r2) -> Optional[Tuple[Tuple[float, float], Tuple[float, float]]]:
        x1, y1 = p1
        x2, y2 = p2
        dx, dy = x2 - x1, y2 - y1
        d = np.sqrt(dx**2 + dy**2)

        # 無解情況
        if d > r1 + r2 or d < abs(r1 - r2) or d == 0:
            return None

        a = (r1**2 - r2**2 + d**2) / (2 * d)
        h_sq = r1**2 - a**2
        if h_sq < 0:
            return None
        h = np.sqrt(h_sq)

        x0 = x1 + a * dx / d
        y0 = y1 + a * dy / d

        rx = -dy * (h / d)
        ry = dx * (h / d)

        return (x0 + rx, y0 + ry), (x0 - rx, y0 - ry)

    # -------------------------------
    # 定位：三邊定位（最小二乘/解聯立）
    # -------------------------------
    def trilateration(self, distances: Dict[str, float]) -> Tuple[Optional[float], Optional[float]]:
        # 至少三顆
        valid_ids = [aid for aid in distances.keys() if aid in self.anchor_positions]
        if len(valid_ids) < 3:
            return None, None

        # 固定取前三顆（你也可以改成挑訊號最穩的三顆）
        anchor_ids = valid_ids[:3]

        A = []
        b = []

        # 用 (0,1) 和 (1,2) 兩個方程組
        for i in range(2):
            x1, y1 = self.anchor_positions[anchor_ids[i]]
            x2, y2 = self.anchor_positions[anchor_ids[i + 1]]
            r1 = distances[anchor_ids[i]]
            r2 = distances[anchor_ids[i + 1]]

            A.append([2 * (x2 - x1), 2 * (y2 - y1)])
            b.append(
                r1 * r1 - r2 * r2
                - x1 * x1 - y1 * y1
                + x2 * x2 + y2 * y2
            )

        try:
            pos = np.linalg.solve(np.array(A, dtype=float), np.array(b, dtype=float))
            return float(pos[0]), float(pos[1])
        except np.linalg.LinAlgError:
            return None, None

    # -------------------------------
    # 視覺：更新 Tag + 軌跡
    # -------------------------------
    def update_tag_visuals(self, x: float, y: float, distances: Dict[str, float]):
        self.tag_point.set_data([x], [y])

        self.trail_x.append(x)
        self.trail_y.append(y)
        if len(self.trail_x) > self.max_trail_points:
            self.trail_x.pop(0)
            self.trail_y.pop(0)
        self.tag_trail.set_data(self.trail_x, self.trail_y)

        # 更新距離文字（顯示在各 anchor 上方）
        for anchor_id, dist in distances.items():
            if anchor_id in self.distance_texts:
                ax, ay = self.anchor_positions[anchor_id]
                self.distance_texts[anchor_id].set_position((ax, ay + 0.2))
                self.distance_texts[anchor_id].set_text(f"{anchor_id}: {dist:.2f}m")

    # -------------------------------
    # 主更新：每一幀讀 UDP 最新封包並更新
    # -------------------------------
    def update_plot(self, frame):
        latest_data = None

        # 把 buffer 內的封包吃到只剩最後一包（避免延遲）
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
                latest_data = data
            except socket.error:
                break

        if latest_data:
            try:
                json_data = json.loads(latest_data.decode("utf-8", errors="ignore"))

                # 解析距離
                distances: Dict[str, float] = {}
                for anchor in json_data.get("anchors", []):
                    anchor_id = anchor.get("id")
                    if anchor_id in self.anchor_positions:
                        try:
                            distances[anchor_id] = float(anchor.get("distance"))
                        except (TypeError, ValueError):
                            pass

                # 先更新文字（即使不足也不報錯）
                for anchor_id in self.anchor_positions.keys():
                    if anchor_id in distances:
                        d = distances[anchor_id]
                        ax, ay = self.anchor_positions[anchor_id]
                        self.distance_texts[anchor_id].set_position((ax, ay + 0.2))
                        self.distance_texts[anchor_id].set_text(f"{anchor_id}: {d:.2f}m")
                    else:
                        # 沒收到就保留 "--"
                        pass

                # 有幾顆就用幾顆算
                n = len(distances)

                if n >= 3:
                    x, y = self.trilateration(distances)
                    if x is not None and y is not None:
                        self.update_tag_visuals(x, y, distances)

                elif n == 2:
                    ids = list(distances.keys())[:2]
                    p1, r1 = self.anchor_positions[ids[0]], distances[ids[0]]
                    p2, r2 = self.anchor_positions[ids[1]], distances[ids[1]]

                    inter = self.intersections(p1, r1, p2, r2)
                    if inter:
                        sol1, sol2 = inter
                        # 你原本想取 y>=0 的那個解
                        x, y = sol1 if sol1[1] >= 0 else sol2
                        self.update_tag_visuals(x, y, distances)

                # n==1：不算 tag，不動就好

            except Exception as e:
                print(f"Error: {e}")

        # blit=True 需要回傳所有 artist
        return (
            *self.anchor_artists,
            self.tag_point,
            self.tag_trail,
            *self.distance_texts.values(),
        )

    def run(self):
        ani = FuncAnimation(
            self.fig,
            self.update_plot,
            interval=16,
            blit=True,
            cache_frame_data=False
        )
        plt.show()


if __name__ == "__main__":
    system = UWBPositionSystem()
    ani = FuncAnimation(
        system.fig,
        system.update_plot,
        interval=16,
        blit=True,
        cache_frame_data=False
    )
    plt.show()