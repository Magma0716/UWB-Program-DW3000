// RX

#include "dw3000.h"

#define PIN_RST 27
#define PIN_IRQ 34
#define PIN_SS 4

#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define POLL_TX_TO_RESP_RX_DLY_UUS 500 
#define RESP_RX_TIMEOUT_UUS 1500        
#define STS_OFFSET 10.65  // STS mode 偏差

const uint8_t PAN_ID[] = { 0xCA, 0xDE };     
const uint8_t TAG_ADDR[] = { 'T', '1' };      

// 需要修改!!
#define NUM_ANCHORS 1  // <-- Anchor數量
static const char ANCHOR_LIST[NUM_ANCHORS][2] = { 
    {'A', '1'},  // Anchor 1
    //{'A', '2'},  // Anchor 2
    //{'A', '3'}   // Anchor 3 
};


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
dwt_config_t config = {
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

static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, PAN_ID[0], PAN_ID[1], TAG_ADDR[0], TAG_ADDR[1], 'A', '1', 0xE0, 0, 0};
static uint8_t rx_resp_msg[] = {0x41, 0x88, 0, PAN_ID[0], PAN_ID[1], 'A', '1', TAG_ADDR[0], TAG_ADDR[1], 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t rx_buffer[24];
static uint8_t frame_seq_nb = 0;

bool isExpectedFrame(uint8_t *buffer) {
    uint8_t saved_sn = buffer[2];
    buffer[2] = 0;
    bool match = (memcmp(buffer, rx_resp_msg, 10) == 0);
    buffer[2] = saved_sn;
    return match;
}

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

    // Set expected response's delay and timeout.
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
}

void loop() {
    static float smooth_dist = 0;
    static bool first_run = true;

    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    uint32_t status;
    while (!((status = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {};

    if (status & SYS_STATUS_RXFCG_BIT_MASK) {
        dwt_readrxdata(rx_buffer, 24, 0);
        if (isExpectedFrame(rx_buffer)) {
            
            // 計算 tof 距離
            uint32_t t1, t2, t3, t4;

            /* Read carrier integrator value and calculate clock offset ratio. */
            float ratio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

            /* 時間戳 */
            t1 = dwt_readtxtimestamplo32();
            t2 = dwt_readrxtimestamplo32();
            resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &t3);
            resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &t4);

            double raw = (((t2 - t1) - (t4 - t3) * (1 - ratio)) / 2.0) * DWT_TIME_UNITS * 299792458.0;
            double poll_time_us = (double)(t4 - t3) * DWT_TIME_UNITS * 1e6;
            double resp_time_us = (double)(t2 - t1) * DWT_TIME_UNITS * 1e6;

            if (raw > 5.0) raw -= STS_OFFSET; // STS mode 偏差 (11m) 

            if (raw > 0 && raw < 40.0) {
                if (first_run) { smooth_dist = raw; first_run = false; }
                else { smooth_dist = (smooth_dist * 0.8) + (raw * 0.2); }
            }
            Serial.printf(
                "DATA, %3.2f, %3.2f\n",
                poll_time_us + resp_time_us, raw
            );

        }
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
    } 
    else{
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    }
    frame_seq_nb++;
    delay(20); //normal 200
}
