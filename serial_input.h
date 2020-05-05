#ifdef SERIAL_INPUT

/***************************************************
 *
 * External Button Data structure / Macro
 *
 ***************************************************/
#define CH_DATA_PKT_MAGIC  (0xCAFEBABE)
#define RX_CH_PKT_DATA_TIMEOUT  (3)  // 115200 bps(14400 bytes/sec, 69 us/ch), 57600 bps(7200 bytes/sec, 138 us/ch)

#define MAX_CH  (16)
#define COMMON_CHANNELS (4) /* AETR */
#define EXTENDED_CHANNELS (MAX_CH - COMMON_CHANNELS)

/***************************************************
 *
 * Module : SERIAL_INPUT
 * 
 * Description : Return value of RX_DataPkt()
 *
 ***************************************************/
typedef enum ERR_CH_PKT_DATA { 
  ERR_CH_PKT_DATA_MAGIC_ERROR = -4,
  ERR_CH_PKT_DATA_MINMAX_ERROR = -3,
  ERR_CH_PKT_DATA_CRC_ERROR = -2,
  ERR_CH_PKT_DATA_TIMEOUT = -1,
  ERR_CH_PKT_DATA_VALID = 0,
  ERR_CH_PKT_DATA_XFER_OK = 1,
} eERR_CH_PKT_DATA_t;

struct ChannelData {
  uint8_t ucAileron;
  uint8_t ucElevator;
  uint8_t ucThrottle;
  uint8_t ucRudder;
  uint8_t ucCH[EXTENDED_CHANNELS];
  uint32_t unPktNumber;
};

union CRCData {
    uint32_t ulCRC;
    uint8_t  ucByte[sizeof(uint32_t)];
};

struct ChannelPacketData {
  uint32_t unMagic;
  struct ChannelData stChannelData;
  union CRCData uCRCData;
};

#define CHANNEL_PACKET_DATA_SIZE  sizeof(struct ChannelPacketData)

union   ChannelPacketData_b {
  struct ChannelPacketData stChannelPacketData;
  uint8_t ucByte[CHANNEL_PACKET_DATA_SIZE];
};


/***************************************************
 *
 * External function
 * 
 ***************************************************/

extern int RX_ChannelPktData(void);

#endif /* SERIAL_INPUT */
