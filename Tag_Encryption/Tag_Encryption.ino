// RX

#include "dw3000.h"
#include "dw3000_mac_802_15_4.h"

/* ================================ */
/* ========== 數據修改區 =========== */
/* ================================ */

// 延遲時間
#define POLL_TX_TO_RESP_RX_DLY_UUS 500  // Tround (未加密:240, 加密:500)
#define RESP_RX_TIMEOUT_UUS 1500        // T4 (未加密:400, 加密:1500)
 
// Tag 強迫休息時間
#define RNG_DELAY_MS 200  // <-- 改小能讓輸出變快

// Anchor 數量
#define NUM_ANCHORS 1

// STS 加密 (for PHR ms)
#define STS_ENCRYPTION false

// AES 加密 (for Payload distance)
#define AES_ENCRYPTION true


/* ================================ */
/* ===== DW3000 Basic Config ====== */
/* ================================ */

#define PIN_RST 27
#define PIN_IRQ 34
#define PIN_SS 4
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define FRAME_LEN 24
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
     
#define STS_OFFSET 10.65  // STS mode 偏差

const uint8_t PAN_ID[] = { 0xCA, 0xDE };     
const uint8_t TAG_ADDR[] = { 'T', '1' };      
extern dwt_txconfig_t txconfig_options;

/* ================================ */
/* ===== Anchor List Settings ===== */
/* ================================ */

#if NUM_ANCHORS == 1
    static const char ANCHOR_LIST[NUM_ANCHORS][2] = {
        {'A', '1'},  // Anchor 1
    };
#elif NUM_ANCHORS == 2
    static const char ANCHOR_LIST[NUM_ANCHORS][2] = {
        {'A', '1'},  // Anchor 1
        {'A', '2'},  // Anchor 2
    };
#elif NUM_ANCHORS == 3
    static const char ANCHOR_LIST[NUM_ANCHORS][2] = {
        {'A', '1'},  // Anchor 1
        {'A', '2'},  // Anchor 2
        {'A', '3'}   // Anchor 3
    };
#endif


/* ================================ */
/* ======== STS Encryption ======== */
/* ================================ */

#if STS_ENCRYPTION == false
    /*
    (未加密)
    Default communication configuration. We use default non-STS DW mode.
    */
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
#else
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
#endif


/* ================================ */
/* =========== Message ============ */
/* ================================ */

/* Messages */
static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, PAN_ID[0], PAN_ID[1], TAG_ADDR[0], TAG_ADDR[1], 'A', '1', 0xE0, 0, 0};
static uint8_t rx_resp_msg[] = {0x41, 0x88, 0, PAN_ID[0], PAN_ID[1], 'A', '1', TAG_ADDR[0], TAG_ADDR[1], 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t rx_buffer[FRAME_LEN];

/* Initiator data */
#define DEST_ADDR       0x1122334455667788 /* this is the address of the responder */
#define SRC_ADDR        0x8877665544332211 /* this is the address of the initiator */
#define DEST_PAN_ID     0x4321             /* this is the PAN ID used in this example */


/* Frame counter */
static uint32_t frame_seq_nb = 0;


/* ================================ */
/* ========= AES Settings ========= */
/* ================================ */

/* AES Configuration */
static dwt_aes_config_t aes_config = {
  AES_key_RAM,
  AES_core_type_CCM,
  MIC_0,
  AES_KEY_Src_Register,
  AES_KEY_Load,
  0,
  AES_KEY_128bit,
  AES_Encrypt
};

/* AES KEY */
static dwt_aes_key_t    keys_options[NUM_OF_KEY_OPTIONS]=
{
    {0x00010203, 0x04050607, 0x08090A0B, 0x0C0D0E0F, 0x00000000, 0x00000000, 0x00000000, 0x00000000},
    {0x11223344, 0x55667788, 0x99AABBCC, 0xDDEEFF00, 0x00000000, 0x00000000, 0x00000000, 0x00000000},
    {0xFFEEDDCC, 0xBBAA9988, 0x77665544, 0x33221100, 0x00000000, 0x00000000, 0x00000000, 0x00000000}
};

/* AES buffers */
static uint8_t aes_tx_buffer[32];
static uint8_t aes_rx_buffer[32];

/* AES jobs */
static dwt_aes_job_t aes_job_tx, aes_job_rx;

/* mac format */
mac_frame_802_15_4_format_t     mac_frame= {
  {
    {0x09, 0xEC},
    0x00,
    {0x21, 0x43},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    { 0x0F, {0x00, 0x00, 0x00, 0x00}, 0x00 }
  },
  0x00
};


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
    digitalWrite(PIN_RST, LOW); 
    delay(10); 
    digitalWrite(PIN_RST, HIGH); 
    delay(100);

    /* DW3000 INIT */
    dwt_initialise(DWT_DW_INIT);
    dwt_configure(&config);

    /* STS */
    if(STS_ENCRYPTION){
        static dwt_sts_cp_key_t sts_key = { 0x12345678, 0x9ABCDEF0, 0x24681357, 0x13572468 };
        static dwt_sts_cp_iv_t sts_iv =   { 0x11223344, 0x55667788, 0x9900AABB };
        dwt_configurestskey(&sts_key);
        dwt_configurestsiv(&sts_iv);
        dwt_configurestsloadiv();
    };

    /* AES */
    if(AES_ENCRYPTION){
        /* Configure the TX spectrum parameters (power, PG delay and PG count) */
        dwt_configuretxrf(&txconfig_options);

        /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug */
        dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

        aes_job_tx.mode        = AES_Encrypt;         /* this is encryption job */
        aes_job_tx.src_port    = AES_Src_Tx_buf;      /* dwt_do_aes will take plain text to the TX buffer */
        aes_job_tx.dst_port    = AES_Dst_Tx_buf;      /* dwt_do_aes will replace the original plain text TX buffer with encrypted one */
        aes_job_tx.nonce       = nonce;               /* pointer to the nonce structure*/
        aes_job_tx.header      = (uint8_t *)MHR_802_15_4_PTR(&mac_frame); /* plain-text header which will not be encrypted */
        aes_job_tx.header_len  = MAC_FRAME_HEADER_SIZE(&mac_frame);
        aes_job_tx.payload     = tx_poll_msg;         /* payload to be encrypted */
        aes_job_tx.payload_len = sizeof(tx_poll_msg); /* size of payload to be encrypted */

        aes_job_rx.mode        = AES_Decrypt;      /* this is decryption job */
        aes_job_rx.src_port    = AES_Src_Rx_buf_0; /* The source of the data to be decrypted is the IC RX buffer */
        aes_job_rx.dst_port    = AES_Dst_Rx_buf_0; /* Decrypt the encrypted data to the IC RX buffer : this will destroy original RX frame */
        aes_job_rx.header_len  = aes_job_tx.header_len;
        aes_job_rx.header      = aes_job_tx.header;/* plain-text header which will not be encrypted */
        aes_job_rx.payload     = rx_buffer;        /* pointer to where the decrypted data will be copied to when read from the IC*/
    }

    /* Antenna delay */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* 設定 RX 延遲及 timeout */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
}

void loop() {

    /* 距離設定 */
    static float smooth_dist = 0;
    static bool first_run = true;

    /* 發送 Poll */
    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

    /* AES TX */
    if(AES_ENCRYPTION){
        uint8_t nonce[13] = {0};
        memcpy(nonce, TAG_ADDR, 2);            // Tag addr
        memcpy(&nonce[9], &frame_seq_nb, 4);   // frame_seq_nb 放最後 4 bytes

        // 加密 poll
        aes_job_tx.nonce = nonce;
        aes_job_tx.header = tx_poll_msg;
        aes_job_tx.header_len = 10;
        aes_job_tx.payload = tx_poll_msg + 10;
        aes_job_tx.payload_len = sizeof(tx_poll_msg) - 10;

        aes_job_tx.src_port = AES_Src_Scratch; // 來源 buffer
        aes_job_tx.dst_port = AES_Dst_Tx_buf;  // 寫到 TX buffer
        
        aes_job_tx.mode = AES_Encrypt;
        aes_job_tx.mic_size = MIC_16;
        aes_job_tx.payload  = aes_tx_buffer;

        dwt_do_aes(&aes_job_tx, aes_config.aes_core_type);
    }
    else{
        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
    }

    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    uint32_t status;
    while (!((status = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {};

    if (status & SYS_STATUS_RXFCG_BIT_MASK) {
        
        /* 解密 respone */
        if(AES_ENCRYPTION){
            dwt_readrxdata(aes_rx_buffer, FRAME_LEN, 0);

            aes_job_rx.nonce = (uint8_t*)&frame_seq_nb;
            aes_job_rx.header = aes_rx_buffer;
            aes_job_rx.header_len = 10;
            aes_job_rx.payload = aes_rx_buffer + 10;
            aes_job_rx.payload_len = FRAME_LEN - 10;
            aes_job_rx.src_port = AES_Src_Rx_buf_0;
            aes_job_rx.dst_port = AES_Dst_Rx_buf_0;
            aes_job_rx.mode = AES_Decrypt;
            aes_job_rx.mic_size = MIC_8;
            aes_job_rx.payload  = rx_buffer;

            dwt_do_aes(&aes_job_tx, aes_config.aes_core_type);
        }
        else{
            dwt_readrxdata(rx_buffer, FRAME_LEN, 0);
        }

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
    delay(RNG_DELAY_MS);
}
