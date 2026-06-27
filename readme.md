# DW3000 Programming

## [Arduino] DW3000 資料夾
需要把 **DW3000資料夾**，放入到 **libraries資料夾** 裡面
```
 .\Arduino\libraries 
```
Arduino 內的設定為：

![board_setting](.\Draw_Error_Images\markdown_image\板子設定.png)



## [Python] 插件
Python 程式需要的插件
```bash
pip install -r requirements.txt
```





## Tag_Encryption\.ino
自訂義區域：
* **環境**：總共有多少板子在跑
```cpp
// Tag 數量
int totalTags = 4;  // 1 ~ 4 

// Anchor 數量
#define NUM_ANCHORS 4   // 1 ~ 4 
```



* **狀態**
```cpp
// Tag 名稱
const uint8_t TAG_ADDR[] = { 'T', '1' };  // 'T', ('1', '2', '3', '4') 

// STS 加密 
#define STS_ENCRYPTION false  // true, false

// AES 加密 
#define AES_ENCRYPTION false  // true, false

// Padding
#define Padding 0  // 0 ~ 47

// Nonce (IV)
#define Random_Nonce_Byte 0  // 0 ~ 4
```



* **網路**：網路名稱 "SSID"、網路密碼 "PASSWORD"
```cpp
// Wifi
#define tmp_ssid "SSID"
#define tmp_password "PASSWORD"
```



* **窗口**：如果 Tag 總數超過1個，window_mode 需設為 true
```cpp
// 每個 tag 窗口時間
unsigned long slotDuration = 30;
int myTagID = (int)TAG_ADDR[1] - '0';

#define window_mode true
```





## Anchor_Encryption\.ino
自訂義區域：

* **狀態**
```cpp
// Anchor 名稱
const uint8_t ANCHOR_ADDR[] = { 'A', '1' };  // 'A', ('1', '2', '3', '4') 

// STS 加密 (for PHR ms)
#define STS_ENCRYPTION false  // true, false

// AES 加密 (for Payload distance)
#define AES_ENCRYPTION false  // true, false

// Padding
#define Padding 0
```





## 2D_position_display\.py
只有2D定位

## Draw_dis_ns\.py (畫距離、延遲圖)
自訂義區域：
```py
PORT = 'COM10'       # 改成插孔的 port 號
DATA_LIMIT = 5000    # 資料筆數
padding = '0'        # 0 ~ 47
encryption = 'AES_'  # 加密狀態分別有 non-, STS_, AES_, AES+STS_
```


## 2D_3D_position_display_Draw_res_jitter_ns\.py (畫殘差、跳動、延遲圖)
自訂義區域：
```py
CONFIG = {
    "ENABLE_STATS_EXPORT": True,    # 是否計算並輸出 Residual/Jump 圖表與存檔
    "SHOW_CLOUD_POINTS": True,      # 初始狀態：歷史路徑雲 ('x' 點)
    "SHOW_RAW_POINTS": True,        # 初始狀態：當前原始測量點 (Raw Data)
    "SHOW_PREDICT_POINTS": False,   # 初始狀態：EKF 預測後的點與連線
    "TARGET_SAMPLES": 1000,         # 達到多少樣本後自動存檔
}
```




## Draw_res_jitter_ns (多Tag整合圖片)
需要自己輸入資料數值，分別有Residual、Jitter、Latency：
```py
data = 
{
    'Tags':     [1, 2, 2, 3, 3, 3, 4, 4, 4, 4],
    'Residual': [0.0164, 0.0171, 0.0197, 0.0196, 0.0194, 0.0517, 0.0336, 0.0200, 0.0783, 0.0880],
    'Jitter':   [0.0336, 0.0485, 0.0332, 0.0386, 0.0322, 0.0724, 0.0410, 0.0341, 0.1503, 0.0966],
    'Latency':  [2.29, 2.30, 2.29, 2.30, 2.33, 2.34, 2.34, 2.35, 2.37, 2.31]
}
```
自訂義區域：

```py
section = 0
choose = 0
```
![non-encryption](.\Draw_Error_Images\markdown_image\未加密.png)


```py
section = 0
choose = 1
```
![encryption](.\Draw_Error_Images\markdown_image\AES加密.png)


```py
section = 1
# choose = X
```
![both](.\Draw_Error_Images\markdown_image\未加密_AES加密.png)




## Positioning\.ino (佈置圖片)
自訂義區域：
```py
# 是否加入Tag?
Add_Tag = False 
```
![no Tag](.\Draw_Error_Images\markdown_image\3D_Layout.png)

```py
# 是否加入Tag?
Add_Tag = True 
```
![Tag](.\Draw_Error_Images\markdown_image\3D_Layout_with_1tag.png)


---

## 其他零碎的程式

* UDP_to_Wireshark：接收 Tag 到 Anchor 之間的封包
* Draw_packet：畫出 poll 或是 respone 的封包格式
* Draw_Error_Images：儲存統計圖片的資料夾