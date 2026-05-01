#include "dw3000.h"
#include "dw3000_mac_802_15_4.h"
#include <WiFi.h>
#include <WiFiUdp.h>

/* ================================ */
/* ========== 數據修改區 =========== */
/* ================================ */

// WiFi 設定
const char* ssid = "Alan6711";
const char* password = "bbb520111";
WiFiUDP udp;
const IPAddress BROADCAST_IP(255, 255, 255, 255);
const int UDP_PORT = 8001;

// 系統參數
#define NUM_ANCHORS 4
#define RNG_DELAY_MS 20      // 測距間隔
#define AES_ENCRYPTION true   // 必須與 Anchor 一致
#define STS_ENCRYPTION false

// 延遲時間對齊 (重要：與 Anchor 的 2000us 對齊)
#define POLL_TX_TO_RESP_RX_DLY_UUS 1800 
#define RESP_RX_TIMEOUT_UUS 500
#define RESP_MSG_POLL_RX_TS_IDX 0        
#define RESP_MSG_RESP_TX_TS_IDX 4        

/* ================================ */
/* ===== DW3000 硬體與地址 ======== */
/* ================================ */

#define PIN_RST 27
#define PIN_IRQ 34
#define PIN_SS 4
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

const uint8_t PAN_ID[] = { 0xDE, 0xCA }; // 統一使用 LE 格式
const uint8_t TAG_ADDR[] = { 'T', '1' };
const uint8_t ANCHOR_LIST[NUM_ANCHORS][2] = {{'A','1'}, {'A','2'}, {'A','3'}, {'A','4'}};
int current_anchor = 0;

static uint8_t rx_buffer[127];
static uint8_t nonce[13];
static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, PAN_ID[0], PAN_ID[1], TAG_ADDR[0], TAG_ADDR[1], 0, 0, 0xE0, 0, 0};

mac_frame_802_15_4_format_t mac_frame = {
    {{0x09, 0xEC}, 0x00, {PAN_ID[0], PAN_ID[1]}, {TAG_ADDR[0], TAG_ADDR[1]}, {0,0}, {0x0F, {0,0,0,0}, 0x00}}, 0x00
};

static dwt_aes_config_t aes_config = {
    AES_key_RAM, AES_core_type_CCM, MIC_0, AES_KEY_Src_Register, AES_KEY_Load, 0, AES_KEY_128bit, AES_Encrypt
};

static dwt_aes_key_t key = {0x00010203, 0x04050607, 0x08090A0B, 0x0C0D0E0F};
static dwt_aes_job_t aes_job_tx, aes_job_rx;

/* ================================ */
/* =========== 核心邏輯 =========== */
/* ================================ */

void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    udp.begin(UDP_PORT);

    spiBegin(PIN_IRQ, PIN_RST);
    spiSelect(PIN_SS);
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) { while(1); }
    

    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    // AES Job 初始化
    aes_job_tx = {AES_Encrypt, AES_Src_Tx_buf, AES_Dst_Tx_buf, nonce, (uint8_t *)&mac_frame, 21, tx_poll_msg, sizeof(tx_poll_msg), 0};
    aes_job_rx = {AES_Decrypt, AES_Src_Rx_buf_0, AES_Dst_Rx_buf_0, nonce, (uint8_t *)&mac_frame, 21, rx_buffer, sizeof(rx_buffer), 0};
}

void loop() {
    // 切換目標 Anchor
    tx_poll_msg[7] = ANCHOR_LIST[current_anchor][0];
    tx_poll_msg[8] = ANCHOR_LIST[current_anchor][1];
    mac_frame.mhr_802_15_4.dest_addr[0] = tx_poll_msg[7];
    mac_frame.mhr_802_15_4.dest_addr[1] = tx_poll_msg[8];

    // 生成隨機 IV 並加密
    uint32_t rv = esp_random();
    memcpy(mac_frame.mhr_802_15_4.aux_security.frame_counter, &rv, 4);
    mac_frame_get_nonce(&mac_frame, nonce);
    dwt_set_keyreg_128(&key);
    dwt_configure_aes(&aes_config);
    dwt_do_aes(&aes_job_tx, aes_config.aes_core_type);

    dwt_writetxfctrl(aes_job_tx.header_len + aes_job_tx.payload_len, 0, 1);
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    uint32_t status;
    while (!((status = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)));

    if (status & SYS_STATUS_RXFCG_BIT_MASK) {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
        dwt_readrxdata(rx_buffer, dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK, 0);
        
        dwt_do_aes(&aes_job_rx, aes_config.aes_core_type);

        uint32_t t1 = dwt_readtxtimestamplo32();
        uint32_t t2 = dwt_readrxtimestamplo32();
        uint32_t t3, t4;
        resp_msg_get_ts(&rx_buffer[0], &t3);
        resp_msg_get_ts(&rx_buffer[4], &t4);

        float ratio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);
        double dist = (((t2 - t1) - (t4 - t3) * (1 - ratio)) / 2.0) * DWT_TIME_UNITS * 299792458.0;

        if (dist > 0 && dist < 100) {
            char json[100];
            snprintf(json, sizeof(json), "{\"a\":\"%c%c\",\"d\":%.2f}", ANCHOR_LIST[current_anchor][0], ANCHOR_LIST[current_anchor][1], dist);
            Serial.println(json);
            udp.beginPacket(BROADCAST_IP, UDP_PORT);
            udp.write((uint8_t*)json, strlen(json));
            udp.endPacket();
        }
    }
    
    current_anchor = (current_anchor + 1) % NUM_ANCHORS;
    delay(RNG_DELAY_MS);
}