// TX

#include "dw3000.h"

#define PIN_RST 27
#define PIN_IRQ 34
#define PIN_SS 4
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define POLL_RX_TO_RESP_TX_DLY_UUS 1000 

const uint8_t PAN_ID[] = { 0xCA, 0xDE };     
const uint8_t ANCHOR_ADDR[] = { 'A', '1' };   
const uint8_t TAG_ADDR[] = { 'T', '1' };      

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

/* 
(加密)
Configuration option 33. 
Channel 5, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 128
*/
static dwt_config_t config = {
    5,                 // Channel number. 
    DWT_PLEN_128,      // Preamble length. Used in TX only. 
    DWT_PAC8,          // Preamble acquisition chunk size. Used in RX only. 
    9,                 // TX preamble code. Used in TX only. 
    9,                 // RX preamble code. Used in RX only. 
    3,                 // 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type 
    DWT_BR_6M8,        // Data rate. 
    DWT_PHRMODE_STD,   // PHY header mode. 
    DWT_PHRRATE_STD,   // PHY header rate. 
    129,               // SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. 
    DWT_STS_MODE_1,    // Mode 1 STS enabled 
    DWT_STS_LEN_128,   // (STS length  in blocks of 8) - 1
    DWT_PDOA_M0        // PDOA mode off 
};

static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, PAN_ID[0], PAN_ID[1], TAG_ADDR[0], TAG_ADDR[1], ANCHOR_ADDR[0], ANCHOR_ADDR[1], 0xE0, 0, 0};
static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, PAN_ID[0], PAN_ID[1], ANCHOR_ADDR[0], ANCHOR_ADDR[1], TAG_ADDR[0], TAG_ADDR[1], 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t rx_buffer[24];

void setup() {
    Serial.begin(115200);
    spiBegin(PIN_IRQ, PIN_RST);
    spiSelect(PIN_SS);
    pinMode(PIN_RST, OUTPUT);
    digitalWrite(PIN_RST, LOW); delay(10); digitalWrite(PIN_RST, HIGH); delay(100);

    /* STS加密 */

    dwt_initialise(DWT_DW_INIT);
    static dwt_sts_cp_key_t sts_key = { 0x12345678, 0x9ABCDEF0, 0x24681357, 0x13572468 };
    static dwt_sts_cp_iv_t sts_iv = { 0x11223344, 0x55667788, 0x9900AABB };
    dwt_configurestskey(&sts_key);
    dwt_configurestsiv(&sts_iv);
    dwt_configurestsloadiv();
    dwt_configure(&config);

    /* STS加密 */

    // Apply default antenna delay value.
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
}

void loop() {
    /* Activate reception immediately. */
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    uint32_t status;
    while (!((status = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR))) {};

    if (status & SYS_STATUS_RXFCG_BIT_MASK) {
        // 檢查加密鎖定
        if (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_CP_LOCK_BIT_MASK)) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
            dwt_configurestsloadiv(); // 重整STS
            return;
        }

        dwt_readrxdata(rx_buffer, 24, 0);
        rx_buffer[2] = 0;
        if (memcmp(rx_buffer, rx_poll_msg, 10) == 0) {
            uint64_t rx_ts = get_rx_timestamp_u64();
            uint32_t tx_time = (rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
            dwt_setdelayedtrxtime(tx_time);

            uint64_t tx_ts = (((uint64_t)(tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
            resp_msg_set_ts(&tx_resp_msg[10], rx_ts);
            resp_msg_set_ts(&tx_resp_msg[14], tx_ts);

            dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
            dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);
            if (dwt_starttx(DWT_START_TX_DELAYED) == DWT_SUCCESS) {
                while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK));
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
            }
        }
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
    } else {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
    }
}