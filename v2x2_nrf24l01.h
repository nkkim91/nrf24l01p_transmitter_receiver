#ifdef V2X2_MODEL

#define NUM_CHN (16)

//*******************
//***  AUX flags  ***
//*******************
#define GET_FLAG(ch, mask) ( ch ? mask : 0)
#define CH5_SW  (Channel_AUX & _BV(0))
#define CH6_SW  (Channel_AUX & _BV(1))
#define CH7_SW  (Channel_AUX & _BV(2))
#define CH8_SW  (Channel_AUX & _BV(3))
#define CH9_SW  (Channel_AUX & _BV(4))
#define CH10_SW (Channel_AUX & _BV(5))
#define CH11_SW (Channel_AUX & _BV(6))
#define CH12_SW (Channel_AUX & _BV(7))
#define CH13_SW (Channel_data[CH13]>CHANNEL_SWITCH)
#define CH14_SW (Channel_data[CH14]>CHANNEL_SWITCH)
#define CH15_SW (Channel_data[CH15]>CHANNEL_SWITCH)
#define CH16_SW (Channel_data[CH16]>CHANNEL_SWITCH)

//Bind flag
#define BIND_IN_PROGRESS    protocol_flags &= ~_BV(7)
#define BIND_DONE           protocol_flags |= _BV(7)
#define IS_BIND_DONE        ( ( protocol_flags & _BV(7) ) !=0 )
#define IS_BIND_IN_PROGRESS ( ( protocol_flags & _BV(7) ) ==0 )

enum {
    V2X2    = 0,
    JXD506  = 1
};

// Packet ack status values
enum {
    PKT_PENDING = 0,
    PKT_ACKED,
    PKT_TIMEOUT
};

#define V2X2_BIND_COUNT 1000
// Timeout for callback in uSec, 4ms=4000us for V202
#define V2X2_PACKET_PERIOD 4000
//
// Time to wait for packet to be sent (no ACK, so very short)
#define V2X2_PACKET_CHKTIME  100
#define V2X2_PAYLOADSIZE 16

// 
enum {
  V2X2_FLAG_CAMERA = 0x01, // also automatic Missile Launcher and Hoist in one direction
  V2X2_FLAG_VIDEO  = 0x02, // also Sprayer, Bubbler, Missile Launcher(1), and Hoist in the other dir.
  V2X2_FLAG_FLIP   = 0x04,
  V2X2_FLAG_UNK9   = 0x08,
  V2X2_FLAG_LIGHT  = 0x10,
  V2X2_FLAG_UNK10  = 0x20,
  V2X2_FLAG_BIND   = 0xC0,
  // flags going to byte 10
  V2X2_FLAG_HEADLESS  = 0x02,
  V2X2_FLAG_MAG_CAL_X = 0x08,
  V2X2_FLAG_MAG_CAL_Y = 0x20,
  V2X2_FLAG_EMERGENCY = 0x80, // JXD-506
  // flags going to byte 11 (JXD-506)
  V2X2_FLAG_START_STOP = 0x40,
  V2X2_FLAG_CAMERA_UP  = 0x01,
  V2X2_FLAG_CAMERA_DN  = 0x02,
};

enum {
  V202_INIT2 = 0,
  V202_INIT2_NO_BIND,//1
  V202_BIND1,//2
  V202_BIND2,//3
  V202_DATA//4
};

#endif /* V2X2_MODEL */
