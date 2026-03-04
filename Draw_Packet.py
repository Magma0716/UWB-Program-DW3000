import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

uwb = "DW3000" # DW1000, DW3000

# Utils
def hex_to_bytes(hex_string):
    hex_string = "".join(hex_string.split())
    return [int(hex_string[i:i+2], 16) for i in range(0, len(hex_string), 2)]

def cut(start_lua, length):
    """
    Lua: arr[1] 開始
    Python: bytes[0] 開始
    """
    py_start = start_lua - 1
    return py_start, length


# Packet structure
def get_packet_structure(b):
    if not b:
        return "Empty", []
    struct = []

    if uwb == "DW1000":
    
        # Blink
        if b[0] == 0xC5:
            struct += [
                ("Mac\nHeader",   1, "#FEFDE8"),
                ("Seq\nNum",      1, "#FEFDE8"),
                ("Tag\nLongAdr",  8, "#FEFDE8"),
                ("Tag\nShortAdr", 2, "#FEFDE8"),
                ("GarbageData",   78,"#F2F2F2"),
                ("CRC",           2, "#FEE8E8")
            ]
            return "Blink", struct

        # Ranging_init (need to fix)
        if len(b) >= 2 and b[0] == 0x41 and b[1] == 0x8C:
            struct += [
                ("Header", 2),
                ("SeqNum", 1),
                ("PanID", 2),
                ("TagLongAdr", 8),
                ("AncShortAdr", 2),
                ("Function", 1),
                ("CRC", 2),
            ]
            remain = len(b) - sum(l for _, l in struct)
            if remain > 0:
                struct.append(("GarbageData", remain))
            return "Ranging_init", struct

        # Short MAC
        if len(b) >= 2 and b[0] == 0x41 and b[1] == 0x88:

            func = b[9]
            encryption = b[10]
            
            # Poll
            if func == 0x00:
                struct += [
                    ("Mac\nHeader",   2,  "#FEFDE8"),
                    ("Seq\nNum",      1,  "#FEFDE8"),
                    ("PanID",         2,  "#FEFDE8"),
                    ("Broadcast",     2,  "#FEFDE8"),
                    ("Tag\nShortAdr", 2,  "#FEFDE8"),
                    ("Func",          1,  "#ECFEE8"),
                    ("Anc\nNum",      1,  "#ECFEE8"),
                    ("Anc\nShortAdr", 2,  "#ECFEE8"),
                    ("Anc\nReply",    2,  "#ECFEE8"),
                    ("GarbageData",   75, "#F2F2F2"),
                    ("CRC",           2,  "#FEE8E8")
                ]
                return "Poll", struct

            # Poll_Ack (need to fix)
            if func == 0x01:
                struct += [
                    ("Header", 2),
                    ("SeqNum", 1),
                    ("PanID", 2),
                    ("TagShortAdr", 2),
                    ("AncShortAdr", 2),
                    ("Function", 1),
                    ("CRC", 2),
                ]
                remain = len(b) - sum(l for _, l in struct)
                if remain:
                    struct.append(("GarbageData", remain))
                return "Poll_Ack", struct

            # Range / Final
            if func == 0x02:
                struct += [
                    ("Mac\nHeader",     2, "#FEFDE8"),
                    ("Seq\nNum",        1, "#FEFDE8"),
                    ("PanID",           2, "#FEFDE8"),
                    ("Broadcast",       2, "#FEFDE8"),
                    ("Tag\nShortAdr",   2, "#FEFDE8"),
                    ("Func",            1, "#ECFEE8"),
                    ("Anc\nNum",        1, "#ECFEE8"),
                    ("Anc\nShortAdr",   2, "#ECFEE8"),
                    ("Tag\nPollSent",   5, "#ECFEE8"),
                    ("Tag\nPollAckRec", 5, "#ECFEE8"),
                    ("Anc\nRangeSent",  5, "#ECFEE8"),
                    ("GarbageData",     62,"#F2F2F2"),
                    ("CRC",             2, "#FEE8E8")
                ]
                return "Range/Final", struct

            # ---------- Range_Report ----------
            
            # 未加密
            if func == 0x03 and encryption == 0x00:
                struct += [
                    ("Mac\nHeader",         2, "#FEFDE8"),
                    ("Seq\nNum",            1, "#FEFDE8"),
                    ("PanID",               2, "#FEFDE8"),
                    ("Broadcast",           2, "#FEFDE8"),
                    ("Tag\nShortAdr",       2, "#FEFDE8"),
                    ("Func",                1, "#ECFEE8"),
                    ("Encryption\n(False)", 1, "#FDE8FE"),
                    ("Length",              1, "#FDE8FE"),
                    ("Chiptext",            4, "#FDE8FE"),
                    ("CRC",                 2, "#FEE8E8")
                ]
                return "Range_Report (Encryption - False)", struct
                
            # 加密
            padding = b[11]-32
            if func == 0x03 and encryption == 0x01:
                if padding == 0:
                    struct += [
                        ("Mac\nHeader",        2, "#FEFDE8"),
                        ("Seq\nNum",           1, "#FEFDE8"),
                        ("PanID",              2, "#FEFDE8"),
                        ("Broadcast",          2, "#FEFDE8"),
                        ("Tag\nShortAdr",      2, "#FEFDE8"),
                        ("Func",               1, "#ECFEE8"),
                        ("Encryption\n(True)", 1, "#FDE8FE"),
                        ("Length",             1, "#FDE8FE"),
                        ("IV",                 12,"#FDE8FE"),
                        ("Tag",                16,"#FDE8FE"),
                        ("Chiptext",           4, "#FDE8FE"),
                        ("CRC",                2, "#FEE8E8")
                    ]
                else:
                    struct += [
                        ("Mac\nHeader",        2, "#FEFDE8"),
                        ("Seq\nNum",           1, "#FEFDE8"),
                        ("PanID",              2, "#FEFDE8"),
                        ("Broadcast",          2, "#FEFDE8"),
                        ("Tag\nShortAdr",      2, "#FEFDE8"),
                        ("Func",               1, "#ECFEE8"),
                        ("Encryption\n(True)", 1, "#FDE8FE"),
                        ("Length",             1, "#FDE8FE"),
                        ("IV",                 12,"#FDE8FE"),
                        ("Tag",                16,"#FDE8FE"),
                        ("Chiptext",           4, "#FDE8FE"),
                        ("Padding",      padding, "#FDE8FE"),
                        ("CRC",                2, "#FEE8E8")
                    ]
                return "Range_Report (Encryption - True)", struct

            return "Range_Failed", [("Data", len(b))]

        return "Unknown", [("Data", len(b))]

    elif uwb == "DW3000":
        
        # Poll
        if b[9] == 0xE0:
            struct += [
                ("Mac\nHeader",   2, "#FEFDE8"),
                ("Seq\nNum",      1, "#FEFDE8"),
                ("Pan\nID",       2, "#FEFDE8"),
                ("Tag\nShortAdr", 2, "#FEFDE8"),
                ("Anc\nShortAdr", 2, "#FEFDE8"),
                ("Function",      1, "#ECFEE8"),
                ("CRC",           2, "#FEE8E8")
            ]
            return "Poll", struct
        
        # Respone
        elif b[9] == 0xE1:
            struct += [
                ("Mac\nHeader",   2, "#FEFDE8"),
                ("Seq\nNum",      1, "#FEFDE8"),
                ("Pan\nID",       2, "#FEFDE8"),
                ("Tag\nShortAdr", 2, "#FEFDE8"),
                ("Anc\nShortAdr", 2, "#FEFDE8"),
                ("Function",      1, "#ECFEE8"),
                ("T2\npoll_rx",   4, "#ECFEE8"),
                ("T3\nresp_tx",   4, "#ECFEE8"),
                ("CRC",           2, "#FEE8E8")
            ]
            return "Respone", struct
        
        else:
            return "Unknown", [("Data", len(b))]
        
    else:
        return "Unknown", [("Data", len(b))]
            
# Drawing
def draw_packet(hex_string):
    b = hex_to_bytes(hex_string)
    pkt_type, struct = get_packet_structure(b)

    fig, ax = plt.subplots(figsize=(16, 5))
    x = 0  # 矩形的起點
    idx = 0

    for name, length, color in struct:
        # --- 核心修正 1：視覺寬度定義 ---
        # 如果是 Garbage Data (62B)，我們縮小寬度，但其他欄位保持原樣
        visual_width = length / 2 if length >= 30 else length
        
        # --- 核心修正 2：移除 x 的額外間距 ---
        # 直接使用 visual_width，不要用 max(visual_width, 1)
        rect = Rectangle(
            (x, 1.84), visual_width, 0.8,
            edgecolor="black", facecolor=color,
            linewidth=1.5  # 加粗邊框可以蓋掉視覺微隙
        )
        ax.add_patch(rect)

        # 處理文字換行
        seg = b[idx : idx + length]
        hex_list = [f"{v:02X}" for v in seg]
        
        if length >= 30:
            # 每 14 個 bytes 換行，視覺上比較美觀
            n = 20
            lines = [" ".join(hex_list[i:i+n]) for i in range(0, len(hex_list), n)]
            text = "\n".join(lines)
        else:
            text = " ".join(hex_list)

        # 文字標註 (調整 Y 軸位置確保不重疊)
        ax.text(x + visual_width/2, 2.7, name, ha="center", va="bottom", fontsize=7, weight="bold")
        ax.text(x + visual_width/2, 2.24, text, ha="center", va="center", fontsize=8, family="monospace")
        ax.text(x + visual_width/2, 1.65, f"{length}B", ha="center", va="top", fontsize=8, color="gray")

        # --- 核心修正 3：精準遞增 x ---
        x += visual_width  # 讓下一個矩形的起點剛好等於上一個的終點
        idx += length

    ax.set_xlim(0, x)
    ax.set_ylim(1.0, 3.5)
    ax.set_title(f"UWB Packet: {pkt_type}", fontsize=14, pad=20)
    ax.axis("off")
    plt.tight_layout()
    plt.show()
# Main
if __name__ == "__main__":
    Ihex = input("Hex Packet: ").strip()
    if len(Ihex) % 2:
        Ihex += "0"
    draw_packet(Ihex)
    
''' 
DW1000
41 88 C6 CA DE FF FF A4 9C 02 01 AA AA 00 E0 ED B3 12 16 86 9E EC 12 00 4C 37 25 13 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 A8 3F

DW3000
41 88 A3 CA DE 54 31 41 31 E0 4B F9
'''
