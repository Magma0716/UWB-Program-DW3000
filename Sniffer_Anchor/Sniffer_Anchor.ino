#include "dw3000.h"

#define PIN_RST 27
#define PIN_IRQ 34
#define PIN_SS 4

/*
(未加密)
Default communication configuration. We use default non-STS DW mode.
*/
/*
static dwt_config_t config = {
    5,                // Channel number.
    DWT_PLEN_128,     // Preamble length. Used in TX only.
    DWT_PAC8,         // Preamble acquisition chunk size. Used in RX only.
    9,                // TX preamble code. Used in TX only.
    9,                // RX preamble code. Used in RX only.
    1,                // 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type
    DWT_BR_6M8,       // Data rate.
    DWT_PHRMODE_STD,  // PHY header mode.
    DWT_PHRRATE_STD,  // PHY header rate.
    129,              // SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only.
    DWT_STS_MODE_OFF, // STS disabled
    DWT_STS_LEN_64,   // STS length see allowed values in Enum dwt_sts_lengths_e
    DWT_PDOA_M0       // PDOA mode off
};
*/

// 加密
static dwt_config_t config = {
    5,                // Channel
    DWT_PLEN_128,     // Preamble Length
    DWT_PAC8,         // PAC
    9,                // TX Code
    9,                // RX Code
    3,                // SFD Type (STS 模式請用 3)
    DWT_BR_6M8,       // Data Rate
    DWT_PHRMODE_STD,  // PHR Mode
    DWT_PHRRATE_STD,  // PHR Rate
    (129 + 8 - 8),    // SFD timeout
    DWT_STS_MODE_1,   // STS Mode 1 (必須與通訊雙方一致)
    DWT_STS_LEN_128,  // STS Length
    DWT_PDOA_M0       // PDOA Off
};

/* STS 加密 */

// KEY
static dwt_sts_cp_key_t sts_key = {0x12345678, 0x9ABCDEF0, 0x24681357, 0x13572468};

// IV
static dwt_sts_cp_iv_t sts_iv = {0x11223344, 0x55667788, 0x9900AABB};

void setup() {
    Serial.begin(115200);
    spiBegin(PIN_IRQ, PIN_RST);
    spiSelect(PIN_SS);
    pinMode(PIN_RST, OUTPUT);
    digitalWrite(PIN_RST, LOW); delay(10); digitalWrite(PIN_RST, HIGH); delay(100);

    if (dwt_initialise(DWT_DW_IDLE) == DWT_ERROR) {
        Serial.println("DW3000 初始化失敗");
        while (1);
    }

    // 混雜模式
    dwt_configureframefilter(DWT_FF_DISABLE, 0); 

    // 加密參數 (解開 PHR)
    dwt_configurestskey(&sts_key);
    dwt_configurestsiv(&sts_iv);
    dwt_configurestsloadiv(); // IV 初始化

    // UWB 參數
    dwt_configure(&config);

    // 接收
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    Serial.println("UWB Sniffer 已啟動，監聽中...");
}

void loop() {
    uint32_t status;
    static uint8_t rx_buffer[1024];
    static int fail_streak = 0;

    status = dwt_read32bitreg(SYS_STATUS_ID);

    // 成功接收封包
    if (status & SYS_STATUS_RXFCG_BIT_MASK) {
        
        if (!(status & SYS_STATUS_CP_LOCK_BIT_MASK)) {
            if (!(status & SYS_STATUS_CP_LOCK_BIT_MASK)) {
                fail_streak++;
                // 只有連續失敗時才重置，避免對抗正確的同步
                if (fail_streak < 5) { 
                    dwt_configurestsloadiv(); 
                }
            } else {
                fail_streak = 0; // 一旦成功就歸零
            }
        }
        
        uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFL_MASK_1023;
        
        if (frame_len <= sizeof(rx_buffer)) {
            dwt_readrxdata(rx_buffer, frame_len, 0);

            // 檢查 STS 狀態
            bool sts_ok = (status & SYS_STATUS_CP_LOCK_BIT_MASK);
            
            // Serial.print("[Sniffer] 收到封包 | 長度: ");
            // Serial.print(frame_len);
            // Serial.print(sts_ok ? " | STS: OK" : " | STS: FAIL (IV未同步)");
            
            // 印出 Hex
            // Serial.print(" | DATA: ");
            for (int i = 0; i < frame_len; i++) {
                Serial.printf("%02X ", rx_buffer[i]);
            }
            Serial.println();
        }

        // 清除狀態並重啟接收
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_CP_LOCK_BIT_MASK);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }
    // 發生錯誤或超時
    else if (status & (SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO)) {
        // 清除錯誤並重啟接收，繼續監聽
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO | SYS_STATUS_CP_LOCK_BIT_MASK);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }
}