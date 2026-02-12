#include "dw3000.h"
#include "SPI.h"

#define PIN_RST 27
#define PIN_IRQ 34
#define PIN_SS 4

extern SPISettings _fastSPI;

static dwt_config_t config = {
    5,                /* Channel number. */
    DWT_PLEN_128,     /* Preamble length. */
    DWT_PAC8,         /* Preamble acquisition chunk size. */
    9,                /* TX preamble code. */
    9,                /* RX preamble code. */
    1,                /* NSS SFD */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (129 + 8 - 8),    /* SFD timeout */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length */
    DWT_PDOA_M0       /* PDOA mode off */
};

void setup() {
  Serial.begin(115200); // 用於 Wireshark 傳輸
  
  _fastSPI = SPISettings(16000000L, MSBFIRST, SPI_MODE0);
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR || dwt_configure(&config) == DWT_ERROR) {
    while (1); 
  }

  /* 關鍵步驟：開啟混雜模式，接收所有封包 */
  dwt_setframefilter(DWT_FF_NOT_ACCEPT);
  
  // 如果你的函式庫支援，直接進入 RX 模式
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

void loop() {
  uint32_t status_reg = dwt_read32bitreg(SYS_STATUS_ID);

  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
    uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    uint8_t rx_buffer[1024];

    if (frame_len > 0 && frame_len <= 1024) {
      dwt_readrxdata(rx_buffer, frame_len, 0);

      // 格式：[長度(2 bytes)][原始資料] -> 方便 PC 端解析
      Serial.write((uint8_t)(frame_len & 0xFF));
      Serial.write((uint8_t)((frame_len >> 8) & 0xFF));
      Serial.write(rx_buffer, frame_len);
    }

    // 清除狀態並重啟接收
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
  } 
  else if (status_reg & SYS_STATUS_ALL_RX_ERR) {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
  }
}