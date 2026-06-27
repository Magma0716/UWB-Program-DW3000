import pandas as pd
import serial
import matplotlib.pyplot as plt
import time
from datetime import datetime
import json
import os
import re

'''
seq: rounded delay
distance_m: distance
interval_ms: raw delay
pad_inferred_from_data: padding
'''

# connect
PORT = 'COM10'
BAUD_RATE = 115200
DATA_LIMIT = 150000 # 資料筆數
padding = '0'
encryption = 'AES_' #  non-, STS_, AES_, AES+STS_

# Json
JSON_FILE = 'AES_RandomIV_Results.json'
if os.path.exists(JSON_FILE):
    with open(JSON_FILE, 'r', encoding='utf-8') as f:
        try:
            stats_data = json.load(f)
        except:
            stats_data = {"ms_raw": {}, "dis_raw": {}}
else:
    stats_data = {"ms_raw": {}, "dis_raw": {}}

pad_key = str(padding)
if pad_key not in stats_data["ms_raw"]:
    stats_data["ms_raw"][pad_key] = []
if pad_key not in stats_data["dis_raw"]:
    stats_data["dis_raw"][pad_key] = []



data = []

ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
print('connect to uwb...')
time.sleep(2)

print('start receive date...')

# STS 防止區
if encryption == 'STS_':
    try:
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            arr = line.split(',')
            print(arr)
    except KeyboardInterrupt:
        print("\n按下 ctrl + C 開始讀取")

for set_idx in range(1):
    data = []    
    count = 1
    packageLoss = 0
    while len(data) < DATA_LIMIT:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            
            '''if line:
                arr = line.split(',')
                print(f'{count} -> {arr}')
                if len(arr) == 3 and arr[0] == 'DATA':#and (float(arr[2]) > 0.5 and float(arr[2]) < 2):
                    try:
                        data.append({
                            'dis': float(arr[2]),
                            'ms':  float(arr[1]) # / 1000
                        })
                        
                        if float(arr[1]) / 1000 > 150:
                            packageLoss += 1
                        
                        count += 1
                    except:
                        print('can\'t received')
                        continue'''
            if line:
                try:
                    # 1. 處理那種 ['{...}', '{...}'] 的怪異格式
                    # 先找到第一個 '{' 和最後一個 '}'
                    start_idx = line.find('{')
                    end_idx = line.rfind('}')
                    
                    if start_idx != -1 and end_idx != -1:
                        json_str = line[start_idx : end_idx + 1]
                        
                        json_str = json_str.replace('"', '"').replace("'", '"')
                        
                        parsed_json = json.loads(json_str)
                        
                        # 3. 提取數據 (針對你的格式: anchors 是一個 list)
                        if 'anchors' in parsed_json and len(parsed_json['anchors']) > 0:
                            anchor = parsed_json['anchors'][0] # 取得第一個 Anchor
                            dist_val = float(anchor['distance'])
                            tof_val = float(anchor['tof'])
                            
                            if count % 10 == 0:
                                print(f'{count} -> Dist: {dist_val}, TOF: {tof_val}')
                            
                            data.append({
                                'dis': dist_val,
                                'ms':  tof_val
                            })
                            
                            # 原本的 packageLoss 邏輯 (判斷 TOF 是否異常)
                            if tof_val / 1000 > 150:
                                packageLoss += 1
                            
                            count += 1
                except Exception as e:
                    # 如果解析失敗，印出錯誤原因以便除錯
                    # print(f"Parse Error at line: {line} | Error: {e}")
                    continue
                
    # read data
    #df = pd.read_csv(r'.\Encryption_YP\UWB_Reports\範例_UWB_Result_ENC_ON_IV_UNK_PAD0_20260115-213740.csv')
    df = pd.DataFrame(data)

    # pre-data
    dist = df['dis']
    distAvg = dist.mean()
    distStd = dist.std()

    intv = df['ms']
    intvAvg = intv.mean()
    intvStd = intv.std()

    stats_data["ms_raw"][pad_key].append(round(float(intvStd), 4))
    stats_data["dis_raw"][pad_key].append(round(float(distStd), 4))
    
    with open(JSON_FILE, 'w', encoding='utf-8') as f:
        json.dump(stats_data, f, indent=4)
    print(f"Set {set_idx} Stats saved to {JSON_FILE}")

    # draw plot
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    # left chart
    ax1.plot(
        df.index, dist, 
        color="#5441ff", marker='o', markersize='4', alpha=0.7, linewidth=1
    )
    ax1.axhline(
        distAvg, 
        color='red', linestyle='--', label=f'Avg: {distAvg:.4f} m'
    )

    ax1.set_title(f"Distance\nAvg={distAvg:.4f} m | Std={distStd:.4f} m | N={len(df)}")
    ax1.set_ylabel("Measured Distance (m)")
    ax1.set_xlabel(f"Sample Sequence Number")
    ax1.set_xticks([0, 25000, 50000, 75000, 100000, 125000, 150000])
    ax1.grid(True, which='both', linestyle='-', alpha=0.2)
    ax1.legend(loc='upper right')
    
    # right chart
    ax2.plot(
        df.index, intv, 
        color='#ffb347', marker='x', markersize=4, alpha=0.8, linewidth=1
    )
    ax2.axhline(
        intvAvg, 
        color='red', linestyle='--', label=f'Avg: {intvAvg:.1f} ns'
    )

    ax2.set_title(f"Latency\nAvg={intvAvg:.4f} ns | Std={intvStd:.4f} ns | N={len(df)}")
    ax2.set_ylabel("Transmission Latency (ns)")
    ax2.set_xlabel(f"Sample Sequence Number")
    ax2.set_xticks([0, 25000, 50000, 75000, 100000, 125000, 150000])
    ax2.grid(True, which='both', linestyle='-', alpha=0.2)
    ax2.legend(loc='upper right')

    plt.tight_layout()

    date = str(datetime.now().strftime('%H%M'))

    plt.savefig(f'.\\Draw_Error_Images\\{encryption}Encryption_{padding}padding_{date}-{set_idx}.png', dpi=300)
    plt.close()

ser.close()
print('receive finish!!')