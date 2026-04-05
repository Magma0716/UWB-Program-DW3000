// RX

#include "dw3000.h"
#include "dw3000_mac_802_15_4.h"

/* ================================ */
/* ========== 數據修改區 =========== */
/* ================================ */

// 延遲時間
#define POLL_TX_TO_RESP_RX_DLY_UUS 1720  // Tround (未加密:240, STS加密:500, AES加密:1720)
#define RESP_RX_TIMEOUT_UUS 250          // T4 (未加密:400, STS加密:1500, AES加密:250)
#define RESP_MSG_POLL_RX_TS_IDX 0        // (未加密:10, AES加密:0)
#define RESP_MSG_RESP_TX_TS_IDX 4        // (未加密:14, AES加密:4)

// Tag 強迫休息時間
#define RNG_DELAY_MS 10  // <-- 改小能讓輸出變快

// Anchor 數量
#define NUM_ANCHORS 4

// STS 加密 (for PHR ms)
#define STS_ENCRYPTION false  // false, true

// AES 加密 (for Payload distance)
#define AES_ENCRYPTION true  // false, true

// Padding
#define Padding 0

// Wifi
#define tmp_ssid "Alan6711"
#define tmp_password "bbb520111"

// position setting
#define UDP_BROADCAST_INTERVAL 100  // Minimum interval between UDP broadcasts (ms)
#define ANCHOR_DATA_TIMEOUT 5000   // Timeout for anchor data in milliseconds

/* ================================ */
/* ===== DW3000 Basic Config ====== */
/* ================================ */

#define PIN_RST 27
#define PIN_IRQ 34
#define PIN_SS 4
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define ALL_MSG_SN_IDX 2
#define START_RECEIVE_DATA_LOCATION     8
#define INITIATOR_KEY_INDEX     1
#define RX_BUF_LEN 127

#define STS_OFFSET 10.65  // STS mode 偏差

const uint8_t PAN_ID[] = { 0xCA, 0xDE };     
const uint8_t TAG_ADDR[] = { 'T', '1' };      
extern dwt_txconfig_t txconfig_options;

static double tof;
static double distance;

/* IV 查表去重複設定 */
#define IV_TABLE_SIZE 5005 
static uint32_t iv_history[IV_TABLE_SIZE];
static uint16_t iv_count = 0;  // 記錄目前已經存了多少筆
static uint16_t write_idx = 0; // 記錄下一次要寫入的位置

// 定位
int current_anchor = 0;

/* Position System Settings */
#define MAX_ANCHORS 10            // Maximum number of anchors to track
#define MIN_ANCHORS_TO_SEND 1     // Minimum anchors required for position calculation
#define MAX_VALID_DISTANCE 8.0    // Maximum valid distance in meters

/* Anchor Data Structure */
struct AnchorData {
    char id[2];        // Anchor ID (2 chars)
    double distance;   // Measured distance
    double tof;       // Time of flight
    unsigned long timestamp; // Last update timestamp
    bool active;      // Whether this anchor is active
};

/* Global Variables for Position System */
static AnchorData anchorArray[MAX_ANCHORS];
static int activeAnchors = 0;
static unsigned long lastBroadcastTime = 0;  // Last UDP broadcast timestamp
static char jsonBuffer[1024];

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
#elif NUM_ANCHORS == 4
    static const char ANCHOR_LIST[NUM_ANCHORS][2] = {
        {'A', '1'},  // Anchor 1
        {'A', '2'},  // Anchor 2
        {'A', '3'},  // Anchor 3
        {'A', '4'}   // Anchor 4
    };
#endif

static int currentAnchorIndex = 0;


/* ================================ */
/* ============= Wifi ============= */
/* ================================ */

#define ENABLE_WIFI

#ifdef ENABLE_WIFI
    #include <WiFi.h>
    #include <WiFiUdp.h>

    /* WiFi Credentials */
    const char* ssid = tmp_ssid;
    const char* password = tmp_password;

    /* UDP Broadcast Settings */
    WiFiUDP udp;
    const int UDP_PORT = 8001;  // Choose a UDP port
    const IPAddress BROADCAST_IP(255, 255, 255, 255);  // Broadcast address
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
/* ========= AES Settings ========= */
/* ================================ */

/* mac format */
#if 0
mac_frame_802_15_4_format_t     mac_frame=
{
    /*
    * Frame control[0] = 0x09 = Data frame, security enabled, PEND not set, no ACK required, PANID compression set to zero (no PANID for source)
    * Frame control[1] = 0xEC = With seq num, no IEs, using extended address, frame ver 2 (IEEE Std 802.15.4)
    */
    .mhr_802_15_4.frame_ctrl[0]=0x09,
    .mhr_802_15_4.frame_ctrl[1]=0xEC,

    /* Sequence number initialize value*/
    .mhr_802_15_4.sequence_num=0x00,

    .mhr_802_15_4.dest_pan_id[0]=0x21,
    .mhr_802_15_4.dest_pan_id[1]=0x43,


    /* Set the Security Control field in the Auxiliary Security Header
    *  Security Control = 0xF:
    *                         Security level: 0x7 = MIC 16 (data confidentiality OFF, data authenticity Yes),
    *                         Key Identifier Mode: 0x1 = key determined from key index field,
    *                         Frame Counter Suppression: 0x0 = has the frame counter and the frame counter generates the nonce.
    *                         ASN in Nonce: 0x0 = frame counter is used to generate the nonce (CCM* nonce = SRC ADDR (8), Frame Counter (4) and Nonce Security Level (1) - set to 0x7 above)
    *  This means that format of the AUX header is Security Control (1 octet) + Fame Counter (4 octets) + Key Identifier (1 octet) = 6 octets
    */
    .mhr_802_15_4.aux_security.security_ctrl=0x0F,

};
#endif
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

#if 0
static dwt_aes_config_t aes_config=
{
    .key_load           = AES_KEY_Load,         // load the key into AES engine see Note 15 below
    .key_size           = AES_KEY_128bit,       // use 128bit key
    .key_src            = AES_KEY_Src_Register, // the key source is IC registers
    .aes_core_type      = AES_core_type_CCM,    // Use CCM core
    .aes_key_otp_type   = AES_key_RAM,
    .key_addr           = 0
};
#endif

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
static uint32_t   frame_cnt=0; 
static uint8_t    seq_cnt=0x0A; /* Frame sequence number, incremented after each transmission. */
uint8_t           nonce[13];
int8_t            status;
uint32_t          status_reg;


/* ================================ */
/* =========== Message ============ */
/* ================================ */

/* Messages */
static uint8_t tx_poll_msg[12 + Padding] = {0x41, 0x88, 0, PAN_ID[0], PAN_ID[1], TAG_ADDR[0], TAG_ADDR[1], 0, 0, 0xE0, 0, 0};
static uint8_t rx_resp_msg[20 + Padding] = {0x41, 0x88, 0, PAN_ID[0], PAN_ID[1], 0, 0, TAG_ADDR[0], TAG_ADDR[1], 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t rx_buffer[RX_BUF_LEN];

/* Initiator data */
#define DEST_ADDR       0x1122334455667788 /* this is the address of the responder */
#define SRC_ADDR        0x8877665544332211 /* this is the address of the initiator */
#define DEST_PAN_ID     0x4321             /* this is the PAN ID used in this example */

/* Frame counter */
static uint32_t frame_seq_nb = 0;

/* function */
bool isExpectedFrame(uint8_t *buffer) {
    uint8_t saved_sn = buffer[2];
    buffer[2] = 0;
    bool match = (memcmp(buffer, rx_resp_msg, 10) == 0);
    buffer[2] = saved_sn;
    return match;
}

void crypto_load(int padding) {
    if (padding <= 0) return;
    
    int base_delay = padding * 50000; // 基礎延遲隨 Padding 線性增加
    int jitter = random(0, padding * 500); // 隨機抖動也隨 Padding 增加
    
    volatile uint32_t count = base_delay + jitter;
    while(count--) { __asm__("nop"); }
}

// 檢查並插入新的 IV 到歷史記錄
bool iv_table_insert_if_new(uint32_t new_iv) {
    // 檢查重複
    for (int i = 0; i < iv_count; i++) {
        if (iv_history[i] == new_iv) return false; 
    }

    iv_history[write_idx] = new_iv;
    write_idx++;

    // 更新目前總筆數
    if (iv_count < IV_TABLE_SIZE) {
        iv_count++;
    }

    // 避免易位
    if (write_idx >= IV_TABLE_SIZE) {
        write_idx = 0; 
    }

    return true; 
}

// 生成不重複隨機 IV 並填入 frame_counter
bool set_unique_random_iv() {
    for (int tries = 0; tries < 100000; tries++) {
        uint32_t r = esp_random();
        if (iv_table_insert_if_new(r)) {
            // 寫入 MAC
            mac_frame.mhr_802_15_4.aux_security.frame_counter[0] = (uint8_t)(r & 0xFF);
            mac_frame.mhr_802_15_4.aux_security.frame_counter[1] = (uint8_t)((r >> 8) & 0xFF);
            mac_frame.mhr_802_15_4.aux_security.frame_counter[2] = (uint8_t)((r >> 16) & 0xFF);
            mac_frame.mhr_802_15_4.aux_security.frame_counter[3] = (uint8_t)((r >> 24) & 0xFF);
            return true;
        }
    }
    return false;
}

/* Function to convert all anchor data to JSON string */
void formatPositionDataToJson(char* jsonBuffer, size_t bufferSize) {
    // Start the JSON object with tag ID
    char tagId[3] = {TAG_ADDR[0], TAG_ADDR[1], 0};  // Convert TAG_ADDR array to string
    snprintf(jsonBuffer, bufferSize, "{\"tag\":\"%s\",\"anchors\":[", tagId);

    // Add each active anchor's data
    bool firstAnchor = true;
    for (int i = 0; i < MAX_ANCHORS; i++) {
        if (anchorArray[i].active) {
            // Check if the anchor data is still valid
            if (millis() - anchorArray[i].timestamp <= ANCHOR_DATA_TIMEOUT) {
                // Add comma if not first anchor
                if (!firstAnchor) {
                    strlcat(jsonBuffer, ",", bufferSize);
                }

                // Create anchor data JSON
                char anchorJson[64];
                snprintf(anchorJson, sizeof(anchorJson),
                        "{\"id\":\"%c%c\",\"distance\":%.2f,\"tof\":%.2f}",
                        anchorArray[i].id[0], anchorArray[i].id[1],
                        anchorArray[i].distance, anchorArray[i].tof);

                strlcat(jsonBuffer, anchorJson, bufferSize);
                firstAnchor = false;
            } else {
                // Mark inactive if timeout
                anchorArray[i].active = false;
                activeAnchors--;
            }
        }
    }

    // Close the JSON array and object
    strlcat(jsonBuffer, "]}", bufferSize);
}

/* Function to update anchor data */
void updateAnchorData(const char* anchorId, double distance, double tof) {
    // Check if distance is valid
    if (distance > MAX_VALID_DISTANCE) {
        Serial.printf("Anchor %c%c distance %.2f m exceeds maximum valid distance\n",
                     anchorId[0], anchorId[1], distance);
        return;  // Skip updating if distance is too large
    }

    // Try to find existing anchor
    for (int i = 0; i < MAX_ANCHORS; i++) {
        if (anchorArray[i].active &&
            anchorArray[i].id[0] == anchorId[0] &&
            anchorArray[i].id[1] == anchorId[1]) {
            // Update existing anchor
            anchorArray[i].distance = distance;
            anchorArray[i].tof = tof;
            anchorArray[i].timestamp = millis();
            return;
        }
    }

    // Find empty slot for new anchor
    for (int i = 0; i < MAX_ANCHORS; i++) {
        if (!anchorArray[i].active) {
            // Add new anchor
            anchorArray[i].id[0] = anchorId[0];
            anchorArray[i].id[1] = anchorId[1];
            anchorArray[i].distance = distance;
            anchorArray[i].tof = tof;
            anchorArray[i].timestamp = millis();
            anchorArray[i].active = true;
            activeAnchors++;
            Serial.printf("New anchor %c%c added, distance: %.2f m\n",
                         anchorId[0], anchorId[1], distance);
            return;
        }
    }
}

/* Function to check and remove invalid anchors */
void cleanupInvalidAnchors() {
    for (int i = 0; i < MAX_ANCHORS; i++) {
        if (anchorArray[i].active) {
            // Check timeout
            if (millis() - anchorArray[i].timestamp > ANCHOR_DATA_TIMEOUT) {
                Serial.printf("Anchor %c%c removed due to timeout\n",
                            anchorArray[i].id[0], anchorArray[i].id[1]);
                anchorArray[i].active = false;
                activeAnchors--;
            }
            // Check distance
            else if (anchorArray[i].distance > MAX_VALID_DISTANCE) {
                Serial.printf("Anchor %c%c removed due to invalid distance: %.2f m\n",
                            anchorArray[i].id[0], anchorArray[i].id[1],
                            anchorArray[i].distance);
                anchorArray[i].active = false;
                activeAnchors--;
            }
        }
    }
}

/* Function to convert ranging data to JSON string */
void formatRangingDataToJson(char* jsonBuffer, size_t bufferSize, const char* anchorName, double distance, double tof) {
    snprintf(jsonBuffer, bufferSize,
             "{\"anchor\":\"%s\",\"distance\":%.2f,\"tof\":%.2f}",
             anchorName, distance, tof);
}

/* Function to broadcast UDP with rate limiting */
void broadcastUDP(const char* jsonData) {
#ifdef ENABLE_WIFI
    unsigned long currentTime = millis();
    if (WiFi.status() == WL_CONNECTED &&
        (currentTime - lastBroadcastTime >= UDP_BROADCAST_INTERVAL)) {
        udp.beginPacket(BROADCAST_IP, UDP_PORT);
        udp.write((const uint8_t*)jsonData, strlen(jsonData));
        udp.endPacket();
        lastBroadcastTime = currentTime;
    }
#endif
}


/* ================================ */
/* ============ Set Up ============ */
/* ================================ */

void setup() {
    Serial.begin(115200);

    #ifdef ENABLE_WIFI
        // Initialize WiFi
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid, password);

        Serial.print("Connecting to WiFi");
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }
        Serial.println();
        Serial.print("Connected to WiFi network with IP Address: ");
        Serial.println(WiFi.localIP());

        // Start UDP
        udp.begin(UDP_PORT);
    #endif

    Serial.println("1. =====================");
    spiBegin(PIN_IRQ, PIN_RST);
    spiSelect(PIN_SS);
    Serial.println("2. =====================");
    pinMode(PIN_RST, OUTPUT);
    digitalWrite(PIN_RST, LOW); 
    delay(10); 
    digitalWrite(PIN_RST, HIGH); 
    delay(100);
    Serial.println("3. =====================");
    /* DW3000 INIT */
    dwt_initialise(DWT_DW_INIT);
    dwt_configure(&config);
    Serial.println("4. =====================");

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

        /* Antenna delay */
        dwt_setrxantennadelay(RX_ANT_DLY);
        dwt_settxantennadelay(TX_ANT_DLY);

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
    else
    {
        /* Antenna delay */
        dwt_setrxantennadelay(RX_ANT_DLY);
        dwt_settxantennadelay(TX_ANT_DLY);

        /* 設定 RX 延遲及 timeout */
        dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
        dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    }

    
}


/* ================================ */
/* ============= Loop ============= */
/* ================================ */

void loop() {

    // 取得目前要測距的 Anchor 名稱 (例如 'A', '1')
    char targetID0 = ANCHOR_LIST[currentAnchorIndex][0];
    char targetID1 = ANCHOR_LIST[currentAnchorIndex][1];

    // 更新加密用的 MAC Frame 目的地 (影響 AES Nonce 與 Header)
    // 這裡我們把目標 ID 填入 DEST_ADDR 的低位元組 (假設高位元組固定)
    mac_frame.mhr_802_15_4.dest_addr[0] = targetID1; // '1'
    mac_frame.mhr_802_15_4.dest_addr[1] = targetID0; // 'A'

    // 更新非加密模式用的 tx_poll_msg (如果 AES 沒開時會用到)
    tx_poll_msg[7] = targetID0;
    tx_poll_msg[8] = targetID1;
    
    // 更新預期接收的 ID (用於後續驗證)
    rx_resp_msg[5] = targetID0;
    rx_resp_msg[6] = targetID1;

    /* 距離設定 */
    static float smooth_dist = 0;
    static bool first_run = true;

    /* AES setting */
    if(AES_ENCRYPTION){

        // 生成不重複隨機 IV 並填入
        if(!set_unique_random_iv()) { 
            Serial.println("Error: IV Table Full!"); 
            while(1); 
        }

        /* Program the correct key to be used */
        dwt_set_keyreg_128(&keys_options[INITIATOR_KEY_INDEX-1]);
        /* Set the key index for the frame */
        MAC_FRAME_AUX_KEY_IDENTIFY_802_15_4(&mac_frame)=INITIATOR_KEY_INDEX;

        /* Update MHR to the correct SRC and DEST addresses and construct the 13-byte nonce
         * (same MAC frame structure is used to store both received data and transmitted data - thus SRC and DEST addresses
         * need to be updated before each transmission */
        mac_frame_set_pan_ids_and_addresses_802_15_4(&mac_frame,DEST_PAN_ID,DEST_ADDR,SRC_ADDR);
        mac_frame_get_nonce(&mac_frame,nonce);

        aes_job_tx.mic_size = mac_frame_get_aux_mic_size(&mac_frame);
        aes_config.mode = AES_Encrypt;
        aes_config.mic  = dwt_mic_size_from_bytes(aes_job_tx.mic_size);
        dwt_configure_aes(&aes_config);

        /* The AES job will take the TX frame data and and copy it to DW IC TX buffer before transmission. See NOTE 7 below. */
        status=dwt_do_aes(&aes_job_tx,aes_config.aes_core_type);
        /* Check for errors */
        if (status<0)
        {
            test_run_info((unsigned char *)"AES length error");
            while (1);/* Error */
        }
        else if (status & AES_ERRORS)
        {
            test_run_info((unsigned char *)"ERROR AES");
            while (1);/* Error */
        }

        /* configure the frame control and start transmission */
        dwt_writetxfctrl(aes_job_tx.header_len + aes_job_tx.payload_len + aes_job_tx.mic_size + FCS_LEN, 0, 1); /* Zero offset in TX buffer, ranging. */

        /* slow */
        //crypto_load(Padding);

        /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
         * set by dwt_setrxaftertxdelay() has elapsed. */
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
        //dwt_starttx(DWT_START_TX_DELAYED);

        /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
        { };

        /* Increment frame sequence number (modulo 256) and frame counter, after transmission of the poll message . */

        MAC_FRAME_SEQ_NUM_802_15_4(&mac_frame)=++seq_cnt;
        mac_frame_update_aux_frame_cnt(&mac_frame,++frame_cnt);
    }
    else
    {
        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
        dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
        //dwt_starttx(DWT_START_TX_DELAYED);
    }

    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {};

    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
        
        /* 解密 respone */
        if(AES_ENCRYPTION){
            /* Clear good RX frame event in the DW IC status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

            /* Read data length that was received */
            uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID)&RXFLEN_MASK;

            /* A frame has been received: firstly need to read the MHR and check this frame is what we expect:
             * the destination address should match our source address (frame filtering can be configured for this check,
             * however that is not part of this example); then the header needs to have security enabled.
             * If any of these checks fail the rx_aes_802_15_4 will return an error
             * */
            aes_config.mode=AES_Decrypt;
            PAYLOAD_PTR_802_15_4(&mac_frame)=rx_buffer;/* Set the MAC pyload ptr */

            /* This example assumes that initiator and responder are sending encrypted data */
            status=rx_aes_802_15_4(&mac_frame,frame_len,&aes_job_rx,sizeof(rx_buffer),keys_options,DEST_ADDR,SRC_ADDR,&aes_config);
            if (status!=AES_RES_OK)
            {
              do {
                switch (status)
                {
                    case AES_RES_ERROR_LENGTH:
                        test_run_info((unsigned char *)"Length AES error");
                        break;
                    case AES_RES_ERROR:
                        test_run_info((unsigned char *)"ERROR AES");
                        break;
                    case AES_RES_ERROR_FRAME:
                        test_run_info((unsigned char *)"Error Frame");
                        break;
                    case AES_RES_ERROR_IGNORE_FRAME:
                        test_run_info((unsigned char *)"Frame not for us");
                        continue;//Got frame not for us
                }
              } while (1);
            }

            /* Check that the frame is the expected response from the companion "SS TWR AES responder" example.
             * ignore the 8 first bytes of the response message as they contain the poll and response timestamps */
            if (memcmp(&rx_buffer[START_RECEIVE_DATA_LOCATION], &rx_resp_msg[START_RECEIVE_DATA_LOCATION],
                    aes_job_rx.payload_len-START_RECEIVE_DATA_LOCATION) == 0)
            {
                uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
                int32_t rtd_init, rtd_resp;
                float clockOffsetRatio ;

                // 計算 tof 距離
                uint32_t t1, t2, t3, t4;

                /* Retrieve poll transmission and response reception timestamps. See NOTE 9 below. */
                t1 = dwt_readtxtimestamplo32();
                t2 = dwt_readrxtimestamplo32();

                /* Read carrier integrator value and calculate clock offset ratio. See NOTE 11 below. */
                clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1<<26);

                /* Get timestamps embedded in response message. */
                resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &t3);
                resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &t4);

                /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
                // rtd_init = resp_rx_ts - poll_tx_ts;
                // rtd_resp = resp_tx_ts - poll_rx_ts;

                tof = (((t2 - t1) - (t4 - t3) * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
                distance = tof * SPEED_OF_LIGHT;
                double poll_time_us = (double)(t4 - t3) * DWT_TIME_UNITS * 1e9;
                double resp_time_us = (double)(t2 - t1) * DWT_TIME_UNITS * 1e9;
                /*
                Serial.printf(
                    "DATA, %3.2f, %3.2f\n",
                    poll_time_us + resp_time_us, distance
                );
                */
                // 在 Serial.printf 之後加入：
                char currentName[3] = { targetID0, targetID1, 0 };
                updateAnchorData(currentName, distance, tof);
                
                // 移動到下一個 Anchor 並進行清理
                cleanupInvalidAnchors();
                
                // 每一輪測距結束後，檢查是否需要送出 JSON
                if (activeAnchors >= MIN_ANCHORS_TO_SEND) {
                    formatPositionDataToJson(jsonBuffer, sizeof(jsonBuffer));
                    Serial.println(jsonBuffer);
                    #ifdef ENABLE_WIFI
                        broadcastUDP(jsonBuffer);
                    #endif
                }
                
                // test_run_info((unsigned char *)dist_str);
            }
        }
        else{
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
                double poll_time_us = (double)(t4 - t3) * DWT_TIME_UNITS * 1e9;
                double resp_time_us = (double)(t2 - t1) * DWT_TIME_UNITS * 1e9;

                if (raw > 5.0 && STS_ENCRYPTION) raw -= STS_OFFSET; // STS mode 偏差 (11m) 

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

        

    } 
    else
    {
        /* Clear RX error/timeout events in the DW IC status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    }
    currentAnchorIndex = (currentAnchorIndex + 1) % NUM_ANCHORS;
    frame_seq_nb++;
    delay(RNG_DELAY_MS);
}
