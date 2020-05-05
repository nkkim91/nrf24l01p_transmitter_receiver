#ifdef EXT_BUTTON

/***************************************************
 *
 * External Button Data structure / Macro
 *
 ***************************************************/

#define EXT_BTN_READY_TIMEOUT (2000)

#define DATA_PKT_MAGIC  (0xDEADBEEF)
#define DATA_PKT_ACK    (0xFEE1CAFE)
#define RX_PKT_TIMEOUT  (100)  // 115200 bps(14400 bytes/sec, 69 us/ch), 57600 bps(7200 bytes/sec, 138 us/ch)

#define MAX_RUDDER_TRIM  (10)
#define MIN_RUDDER_TRIM  (-10)

#define MAX_ELEVATOR_TRIM  (10)
#define MIN_ELEVATOR_TRIM  (-10)

#define MAX_AILERON_TRIM  (10)
#define MIN_AILERON_TRIM  (-10)

#define BUTTON_FLAG_CAMERA_IDX  (1 << 0)
#define BUTTON_FLAG_VIDEO_IDX   (1 << 1)

#define TRIM_AMPLFY_FACTOR  (4)  // 1024 / 256 = 4


/***************************************************
 *
 * Module : EXT_BUTTON
 * Description : Return value of RX_DataPkt()
 *
 ***************************************************/
typedef enum RX_DATA_RET {
  RX_DATA_MINMAX_ERROR = -3,
  RX_DATA_CRC_ERROR = -2,
  RX_DATA_TIMEOUT = -1,
  RX_DATA_VALID = 0
} eRX_DATA_RET_t;


struct TrimData {
  int8_t cRudder;
  int8_t cElevator;
  int8_t cAileron;
};

struct TrimEEPROMData {
  struct TrimData stTrimData;
  union {
    uint32_t ulCRC;
    uint8_t  ucByte[4];
  } u;
};

struct TxData {
  uint32_t unMagic;
  struct TrimData stTrimData;
  uint32_t unOpt1;
  uint32_t unButtonFlag;
  uint32_t unACK;
  union {
    uint32_t ulCRC;
    uint8_t  ucByte[sizeof(uint32_t)];
  } u;
};

#define TX_PACKET_DATA_SIZE  sizeof(struct TxData)

union   TxPacketData {
  struct TxData stTxData;
  uint8_t ucByte[TX_PACKET_DATA_SIZE];
};

/***************************************************
 *
 * External variables
 *
 ***************************************************/

extern union TxPacketData gstTxPacketData, gstRxPacketData, gstTempRxPacketData;
extern eRX_DATA_RET_t geRxDataRet;


#endif /* EXT_BUTTON */
