// TX

#include "dw3000.h"
#include "dw3000_mac_802_15_4.h"

/* ================================ */
/* ========== 數據修改區 =========== */
/* ================================ */

// 延遲時間
#define POLL_RX_TO_RESP_TX_DLY_UUS 1000 // Treply (未加密:600, 加密:1000)

// Anchor 名稱 (e.g. A1, A2, A3...)
const uint8_t ANCHOR_ADDR[] = { 'A', '1' };  

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

const uint8_t PAN_ID[] = { 0xCA, 0xDE };
const uint8_t TAG_ADDR[] = { 'T', '1' };      
extern dwt_txconfig_t txconfig_options;
mac_frame_802_15_4_format_t mac_frame;

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
#endif


/* ================================ */
/* =========== Message ============ */
/* ================================ */

/* Messages */
static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, PAN_ID[0], PAN_ID[1], TAG_ADDR[0], TAG_ADDR[1], ANCHOR_ADDR[0], ANCHOR_ADDR[1], 0xE0, 0, 0};
static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, PAN_ID[0], PAN_ID[1], ANCHOR_ADDR[0], ANCHOR_ADDR[1], TAG_ADDR[0], TAG_ADDR[1], 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t rx_buffer[FRAME_LEN];
static uint8_t received_sn;

#define SRC_ADDR        0x1122334455667788 /* this is the address of the initiator */
#define DEST_ADDR       0x8877665544332211 /* this is the address of the responder */
#define DEST_PAN_ID     0x4321             /* this is the PAN ID used in this example */

/* Frame counter */
static uint32_t frame_seq_nb = 0;


/* ================================ */
/* ========= AES Settings ========= */
/* ================================ */

/* AES Configuration */
static dwt_aes_config_t aes_config= {
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

void setup() {
    Serial.begin(115200);
    test_run_info((unsigned char *)APP_NAME);

    spiBegin(PIN_IRQ, PIN_RST);
    spiSelect(PIN_SS);

    pinMode(PIN_RST, OUTPUT);
    digitalWrite(PIN_RST, LOW); 
    delay(10); 
    digitalWrite(PIN_RST, HIGH); 
    delay(100);

    /* DW3000 INIT*/
    dwt_initialise(DWT_DW_INIT);
    dwt_configure(&config);

    /* STS */
    if(STS_ENCRYPTION){
        static dwt_sts_cp_key_t sts_key = { 0x12345678, 0x9ABCDEF0, 0x24681357, 0x13572468 };
        static dwt_sts_cp_iv_t sts_iv = { 0x11223344, 0x55667788, 0x9900AABB };
        dwt_configurestskey(&sts_key);
        dwt_configurestsiv(&sts_iv);
        dwt_configurestsloadiv();
    }

    /* AES */
    if(AES_ENCRYPTION){
        /* Configure the TX spectrum parameters (power, PG delay and PG count) */
        dwt_configuretxrf(&txconfig_options);

        /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
        * Note, in real low power applications the LEDs should not be used. */
        dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
        
        aes_job_rx.mode        = AES_Decrypt;           /* Mode is set to decryption */
        aes_job_rx.src_port    = AES_Src_Rx_buf_0;      /* Take encrypted frame from the RX buffer */
        aes_job_rx.dst_port    = AES_Dst_Rx_buf_0;      /* Decrypt the frame to the same RX buffer : this will destroy original RX frame */
        aes_job_rx.header_len  = MAC_FRAME_HEADER_SIZE(&mac_frame);  /* Set the header length (mac_frame contains the MAC header) */
        aes_job_rx.header      = (uint8_t *)MHR_802_15_4_PTR(&mac_frame);/* Set the pointer to plain-text header which will not be encrypted */
        aes_job_rx.payload     = rx_buffer;                             /* the decrypted RX MAC frame payload will be read out of IC into this buffer */

        aes_job_tx.mode        = AES_Encrypt;     /* this is encyption job */
        aes_job_tx.src_port    = AES_Src_Tx_buf;  /* dwt_do_aes will take plain text to the TX buffer */
        aes_job_tx.dst_port    = AES_Dst_Tx_buf;  /* dwt_do_aes will replace the original plain text TX buffer with encrypted one */
        aes_job_tx.header_len  = aes_job_rx.header_len;
        aes_job_tx.header      = aes_job_rx.header;/* plain-text header which will not be encrypted */
        aes_job_tx.payload     = tx_resp_msg;      /* payload to be sent */
        aes_job_tx.payload_len = sizeof(tx_resp_msg); /* payload length */
    }
    
    /* antenna delay */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
}

void loop() {

    /* start Rx */
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    uint32_t status;
    while (!((status = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR))) {};

    if (status & SYS_STATUS_RXFCG_BIT_MASK) {
        /* STS lock check */
        if (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_CP_LOCK_BIT_MASK) && STS_ENCRYPTION) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
            dwt_configurestsloadiv(); // 重整 STS 計數器
            return;
        }

        // 先讀整個 RX buffer
        dwt_readrxdata(aes_rx_buffer, FRAME_LEN, 0);

        /* read RX */
        if(AES_ENCRYPTION){

            uint8_t received_sn = aes_rx_buffer[2];

            // AES-CCM 包含地址與計數器
            uint8_t nonce[13] = {0};
            memcpy(nonce, TAG_ADDR, 2);            // Tag addr
            memcpy(&nonce[9], &frame_seq_nb, 4);   // frame_seq_nb 放最後 4 bytes

            // 解密 poll
            aes_job_rx.nonce = nonce;
            aes_job_rx.header = aes_rx_buffer;
            aes_job_rx.header_len = 10;
            aes_job_rx.payload = aes_rx_buffer + 10;
            aes_job_rx.payload_len = FRAME_LEN - 10;
            aes_job_rx.src_port = AES_Src_Rx_buf_0;
            aes_job_rx.dst_port = AES_Dst_Scratch;
            aes_job_rx.mode = AES_Decrypt;

            dwt_do_aes(&aes_job_tx, aes_config.aes_core_type);
        }
        else{
            dwt_readrxdata(rx_buffer, FRAME_LEN, 0);
        }

        /* check poll */
        rx_buffer[2] = 0;

        if (memcmp(rx_buffer, rx_poll_msg, 10) == 0) {
            uint64_t rx_ts = get_rx_timestamp_u64();
            uint32_t tx_time = (rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
            dwt_setdelayedtrxtime(tx_time);

            uint64_t tx_ts = (((uint64_t)(tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
            resp_msg_set_ts(&tx_resp_msg[10], rx_ts);
            resp_msg_set_ts(&tx_resp_msg[14], tx_ts);

            /* 加密 respone */
            if(AES_ENCRYPTION){
                tx_resp_msg[2] = received_sn;

                aes_job_tx.mode = AES_Encrypt;
                aes_job_tx.src_port = AES_Src_Tx_buf;
                aes_job_tx.dst_port = AES_Dst_Tx_buf;
                aes_job_tx.header_len = aes_job_rx.header_len;
                aes_job_tx.header = aes_job_rx.header;        /* plain-text header which will not be encrypted */
                aes_job_tx.payload = tx_resp_msg;             /* payload to be sent */
                aes_job_tx.payload_len = sizeof(tx_resp_msg); /* payload length */

                dwt_do_aes(&aes_job_tx, aes_config.aes_core_type);
            }
            else{
                dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
            }

            dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);

            if (dwt_starttx(DWT_START_TX_DELAYED) == DWT_SUCCESS) {
                while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK));
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
            }

            frame_seq_nb++;
        }
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
    } else {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
    }
}