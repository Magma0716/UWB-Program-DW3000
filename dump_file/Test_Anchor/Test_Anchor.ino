#include "dw3000.h"
#include "dw3000_mac_802_15_4.h"

// 必須與 Tag 對應
#define AES_ENCRYPTION true
#define POLL_RX_TO_RESP_TX_DLY_UUS 2000 
#define RESP_MSG_POLL_RX_TS_IDX 0
#define RESP_MSG_RESP_TX_TS_IDX 4

const uint8_t ANCHOR_ADDR[] = { 'A', '1' }; // 每台 Anchor 改這行 (A1, A2...)
const uint8_t PAN_ID[] = { 0xDE, 0xCA };

#define PIN_RST 27
#define PIN_IRQ 34
#define PIN_SS 4
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

static uint8_t rx_buffer[127];
static uint8_t tx_resp_msg[] = {0,0,0,0,0,0,0,0}; // Payload for timestamps
static uint8_t nonce[13];

mac_frame_802_15_4_format_t mac_frame = {
    {{0x09, 0xEC}, 0x00, {PAN_ID[0], PAN_ID[1]}, {0,0}, {ANCHOR_ADDR[0], ANCHOR_ADDR[1]}, {0x0F, {0,0,0,0}, 0x00}}, 0x00
};

static dwt_aes_config_t aes_config = { AES_key_RAM, AES_core_type_CCM, MIC_0, AES_KEY_Src_Register, AES_KEY_Load, 0, AES_KEY_128bit, AES_Encrypt };
static dwt_aes_key_t key = {0x00010203, 0x04050607, 0x08090A0B, 0x0C0D0E0F};
static dwt_aes_job_t aes_job_tx, aes_job_rx;

void setup() {
    Serial.begin(115200);
    spiBegin(PIN_IRQ, PIN_RST);
    spiSelect(PIN_SS);
    dwt_initialise(DWT_DW_INIT);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setrxantennadelay(RX_ANT_DLY);

    aes_job_tx = {AES_Encrypt, AES_Src_Tx_buf, AES_Dst_Tx_buf, nonce, (uint8_t *)&mac_frame, 21, tx_resp_msg, sizeof(tx_resp_msg), 0};
    aes_job_rx = {AES_Decrypt, AES_Src_Rx_buf_0, AES_Dst_Rx_buf_0, nonce, (uint8_t *)&mac_frame, 21, rx_buffer, sizeof(rx_buffer), 0};
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

void loop() {
    uint32_t status = dwt_read32bitreg(SYS_STATUS_ID);
    if (status & SYS_STATUS_RXFCG_BIT_MASK) {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
        dwt_readrxdata(rx_buffer, dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK, 0);

        // 這裡要做初步過濾，判斷 Poll 是否是找這台 Anchor
        // 在 AES 模式下，地址在解密前的 MAC Header 內
        if (rx_buffer[5] == ANCHOR_ADDR[0] && rx_buffer[6] == ANCHOR_ADDR[1]) {
            // 解析 Tag 傳來的 Nonce 並解密
            memcpy(mac_frame.mhr_802_15_4.aux_security.frame_counter, &rx_buffer[15], 4);
            mac_frame_get_nonce(&mac_frame, nonce);
            dwt_set_keyreg_128(&key);
            dwt_do_aes(&aes_job_rx, aes_config.aes_core_type);

            uint64_t poll_rx_ts = get_rx_timestamp_u64();
            uint32_t resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
            dwt_setdelayedtrxtime(resp_tx_time);
            uint64_t resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

            resp_msg_set_ts(&tx_resp_msg[0], poll_rx_ts);
            resp_msg_set_ts(&tx_resp_msg[4], resp_tx_ts);

            dwt_do_aes(&aes_job_tx, aes_config.aes_core_type);
            dwt_writetxfctrl(aes_job_tx.header_len + aes_job_tx.payload_len, 0, 1);
            
            if (dwt_starttx(DWT_START_TX_DELAYED) == DWT_SUCCESS) {
                while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK));
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
            }
        }
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }
}