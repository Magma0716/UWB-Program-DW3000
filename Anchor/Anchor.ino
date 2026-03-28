#include "dw3000.h"
#include "SPI.h"

// ========= 數據修改區 =========

// 延遲時間
#define POLL_RX_TO_RESP_TX_DLY_UUS 600  // Treply (未加密:600, 加密1000)

// Anchor 名稱 (e.g. A1, A2, A3...)
const uint8_t ANCHOR_ADDR[] = { 'A', '1' };

// STS 加密 (for PHR ms)
#define STS_ENCRYPTION false

// AES 加密 (for Payload distance)
#define AES_ENCRYPTION false

// ==============================


/* TX for Anchor */
const uint8_t PAN_ID[] = { 0xCA, 0xDE };
const uint8_t TAG_ADDR[] = { 'T', '1' };

extern SPISettings _fastSPI;

#define PIN_RST 27
#define PIN_IRQ 34
#define PIN_SS 4

#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4

/* STS Encryption */
#if STS_ENCRYPTION == false
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
      (129 + 8 - 8),    // SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only.
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
      5,                // Channel number. 
      DWT_PLEN_128,     // Preamble length. 
      DWT_PAC8,         // Preamble acquisition chunk size. 
      9,                // TX preamble code. 
      9,                // RX preamble code. 
      3,                // SFD type (4z 8 symbol SDF type)
      DWT_BR_6M8,       // Data rate. 
      DWT_PHRMODE_STD,  // PHY header mode. 
      DWT_PHRRATE_STD,  // PHY header rate. 
      129,// SFD timeout. 
      DWT_STS_MODE_1,   // MODE_1 Payload + STS 
      DWT_STS_LEN_128,  // STS lengths
      DWT_PDOA_M0       // PDOA mode off 
  };
#endif

/* SS-TWR Message Format
 * Poll message from Tag to Anchor:
 * +-----------+--------+----------+-----------+------------+------------+------+--------+
 * | Byte 0-1  | Byte 2 | Byte 3-4 | Byte 5-6  | Byte 7-8   | Byte 9     | 10-11|
 * +-----------+--------+----------+-----------+------------+------------+------+--------+
 * | 0x41, 0x88| Seq    | PAN ID   | Tag ADDR  | Anch ADDR  | 0xE0       | RES  |
 * +-----------+--------+----------+-----------+------------+------------+------+--------+
 */
/* Response message from Anchor to Tag:
 * +-----------+--------+----------+-----------+------------+------------+----------------+----------------+--------+
 * | Byte 0-1  | Byte 2 | Byte 3-4 | Byte 5-6  | Byte 7-8   | Byte 9     | Byte 10-13     | Byte 14-17     | 18-19  |
 * +-----------+--------+----------+-----------+------------+------------+----------------+----------------+--------+
 * | 0x41, 0x88| Seq    | PAN ID   | Anch ADDR | Tag ADDR   | 0xE1       | Poll RX TS     | Resp TX TS     | RES    |
 * +-----------+--------+----------+-----------+------------+------------+----------------+----------------+--------+
 *
 * Field Description:
 * - Frame Control (0x41, 0x88): IEEE 802.15.4 frame control
 * - Seq: Frame sequence number
 * - PAN ID: Network identifier (0xCADE)
 * - Tag/Anch ADDR: Device addresses
 * - 0xE0/0xE1: Message type (Poll/Response)
 * - Poll RX TS: Timestamp of Poll message reception
 * - Resp TX TS: Timestamp of Response message transmission
 * - RES: Reserved bytes
 */


static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, PAN_ID[0], PAN_ID[1], TAG_ADDR[0], TAG_ADDR[1], ANCHOR_ADDR[0], ANCHOR_ADDR[1], 0xE0, 0, 0};
static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, PAN_ID[0], PAN_ID[1], ANCHOR_ADDR[0], ANCHOR_ADDR[1], TAG_ADDR[0], TAG_ADDR[1], 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static uint8_t frame_seq_nb = 0;
static uint8_t rx_buffer[20];
static uint32_t status_reg = 0;
static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;

extern dwt_txconfig_t txconfig_options;

void setup()
{
  UART_init();//Serial.begin(115200);

  /* STS 時間戳加密 */
  if(STS_ENCRYPTION){
    // key
    static dwt_sts_cp_key_t sts_key = {
        0x12345678, 0x9ABCDEF0, 0x24681357, 0x13572468
    };

    // IV
    static dwt_sts_cp_iv_t sts_iv = {
        0x11223344, 0x55667788, 0x9900AABB
    };
    dwt_configurestskey(&sts_key);
    dwt_configurestsiv(&sts_iv);
    dwt_configurestsloadiv();
  }
  dwt_configure(&config);
  
  /* AES 資料加密 */
  /*
  dwt_aes_config_t aes_config = {
    .key = {
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 
        0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10
    },
    .key_index = 1,
    .mic_size = 8,           // 認證碼長度 (4, 8, 16)
    .mode = DWT_AES_CCM_128  // 802.15.4 安全模式
  };
  dwt_configure_aes(&aes_config);
  */

  _fastSPI = SPISettings(16000000L, MSBFIRST, SPI_MODE0);

  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);

  delay(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

  while (!dwt_checkidlerc()) // Need to make sure DW IC is in IDLE_RC before proceeding
  {
    UART_puts("IDLE FAILED\r\n");
    while (1)
      ;
  }

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
  {
    UART_puts("INIT FAILED\r\n");
    while (1)
      ;
  }

  // Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  /* Configure DW IC. See NOTE 6 below. */
  if (dwt_configure(&config)) // if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device
  {
    UART_puts("CONFIG FAILED\r\n");
    while (1)
      ;
  }

  /* Configure the TX spectrum parameters (power, PG delay and PG count) */
  dwt_configuretxrf(&txconfig_options);

  /* Apply default antenna delay value. See NOTE 2 below. */
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
   * Note, in real low power applications the LEDs should not be used. */
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

  Serial.println("Range TX");
  Serial.println("Setup over........");
}

void loop()
{
  /* Activate reception immediately. */
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  /* Poll for reception of a frame or error/timeout. See NOTE 6 below. */
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)))
  {
  };

  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
  {
    // STS 檢查
    /*
    uint32_t status = dwt_read32bitreg(SYS_STATUS_ID);
    if (!(status & SYS_STATUS_CP_LOCK_BIT_MASK)) 
    {
        // Serial.println("STS 驗證失敗！嘗試重置 IV 並丟棄包");
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        dwt_configurestsloadiv(); // 重載 IV 初始值
        return;
    }
    */
    uint32_t frame_len;

    /* Clear good RX frame event in the DW IC status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    /* A frame has been received, read it into the local buffer. */
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len <= sizeof(rx_buffer))
    {
      dwt_readrxdata(rx_buffer, frame_len, 0);

      //if (rx_buffer[9] == 0xE0 && memcmp(&rx_buffer[3], PAN_ID, 2) == 0) {
        
        // 提取發送者 (Tag) 的 ID
        uint8_t incoming_tag_id[2];
        incoming_tag_id[0] = rx_buffer[5]; 
        incoming_tag_id[1] = rx_buffer[6];

        // 將回傳目標設為該 Tag
        tx_resp_msg[7] = incoming_tag_id[0];
        tx_resp_msg[8] = incoming_tag_id[1];
        
        /* Check that the frame is a poll sent by "SS TWR initiator" example.
        * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
        rx_buffer[ALL_MSG_SN_IDX] = 0;
        //if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0) <------
        if(rx_buffer[9] == 0xE0 &&                                
           rx_buffer[3] == PAN_ID[0] && rx_buffer[4] == PAN_ID[1] &&
           rx_buffer[7] == ANCHOR_ADDR[0] && rx_buffer[8] == ANCHOR_ADDR[1])
        {
          uint32_t resp_tx_time;
          int ret;

          /* Retrieve poll reception timestamp. */
          poll_rx_ts = get_rx_timestamp_u64();

          /* Compute response message transmission time. See NOTE 7 below. */
          resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
          dwt_setdelayedtrxtime(resp_tx_time);

          /* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
          resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

          /* Write all timestamps in the final message. See NOTE 8 below. */
          resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
          resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

          /* Write and send the response message. See NOTE 9 below. */
          tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
          dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); /* Zero offset in TX buffer. */
          dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);          /* Zero offset in TX buffer, ranging. */
          ret = dwt_starttx(DWT_START_TX_DELAYED);

          /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 10 below. */
          if (ret == DWT_SUCCESS)
          {
            /* Poll DW IC until TX frame sent event set. See NOTE 6 below. */
            while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
            {
            };

            /* Clear TXFRS event. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

            /* Increment frame sequence number after transmission of the poll message (modulo 256). */
            frame_seq_nb++;
          }
        }
      //}
    }
  }
  else
  {
    /* Clear RX error events in the DW IC status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
  }
}
