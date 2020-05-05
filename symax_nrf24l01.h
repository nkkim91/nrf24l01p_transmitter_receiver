


/***************************************************
 *
 * SYMA X5C Protocol Data structure / Macro
 *
 ***************************************************/
#define BIND_COUNT 345   // 1.5 seconds, was 345
#define FIRST_PACKET_DELAY  12000  // 12 ms, was 12000
#define PACKET_PERIOD       4000     // 4 ms, Timeout for callback in uSec, was 4000
#define INITIAL_WAIT         500  // was 500


#define FIRST_PACKET_RXTX_CHANNEL (8)
#define RXTX_ADDR_SIZE  (5)

/***************************************************
 * Macro below is required for Receiver module 
 ***************************************************/
#ifdef NRF24L01_RECEIVER_MODE

#if defined(CONFIG_SYMA_X5C_RECEIVER_NK) || \
    defined(CONFIG_SYMA_X5C_RECEIVER_CAR2WD_NK)

/* L at the end of constant number is "VERY" important for exact comparing of time */
#define BIND_TIMEOUT  ( 13800L )   /* initial value ==> 1380 ms * 10 = 13.8 sec */
#define BIND_FASTLED_TIMEOUT  ( 4140L )  /* 1380 ms * 3 = 4.14 sec */

#define RECEIVE_PACKET_TIMEOUT  (4)    /* in ms, Shouldn't be too short and too long */

#define RX_RF_ADDR_HOPPING_PERIOD  (32L)  /* Packet transmitted over 4 RF CH in every 4 ms. and RF hopping in every 2 pkt */
#define RX_RF_DATA_HOPPING_PERIOD  (32L)  /* 32 is more balanced. 8 or 16 are ok. but low quality in RX */

#define MIN_RX_ADDR_PKTS  (5) // 2 pkt in 32 ms at best 

#elif defined(CONFIG_HMC5883L_GY273_RECEIVER_NK)

/* L at the end of constant number is "VERY" important for exact comparing of time */
#define BIND_TIMEOUT  ( 41400L )   /* 1380 ms * 30 = 41.4 sec */
#define BIND_FASTLED_TIMEOUT  ( 4140L )  /* 1380 ms * 3 = 4.14 sec */

#define RECEIVE_PACKET_TIMEOUT  (4)    /* in ms, Shouldn't be too short and too long */

#define RX_RF_ADDR_HOPPING_PERIOD  (32L)  /* Packet transmitted over 4 RF CH in every 4 ms. and RF hopping in every 2 pkt */
#define RX_RF_DATA_HOPPING_PERIOD  (32L)  /* 16L or 32L are ok */

#define MIN_RX_ADDR_PKTS  (5) // 2 pkt in 32 ms at best 

#else
#error "## NK - No configuration definition for Bind macro"
#endif

#endif /* NRF24L01_RECEIVER_MODE */

#define FLAG_FLIP      0x01
#define FLAG_VIDEO     0x02
#define FLAG_PICTURE   0x04
#define FLAG_HEADLESS  0x08


#ifdef PROTOCOL_SYMA_X5C_MODEL

#define PKT_IDX_THROTTLE  (0)
#define PKT_IDX_ELEVATOR  (1)
#define PKT_IDX_RUDDER    (2)
#define PKT_IDX_AILERON   (3)

#define RUDDER_ADJUST_IDX    (0)
#define ELEVATOR_ADJUST_IDX  (1)
#define AILERON_ADJUST_IDX   (2)

#define MOTOR_1_IDX      (0)
#define MOTOR_2_IDX      (1)
#define MOTOR_3_IDX      (2)
#define MOTOR_4_IDX      (3)

#define MOTOR_1_MASK     (1 << 0)
#define MOTOR_2_MASK     (1 << 1)
#define MOTOR_3_MASK     (1 << 2)
#define MOTOR_4_MASK     (1 << 3)

#define AILERON_RIGHT    (MOTOR_1 | MOTOR_2)  /* Lower the throttle power of the motor where would like to tilt */
#define AILERON_LEFT     (MOTOR_3 | MOTOR_4)
#define ELEVATOR_UP      (MOTOR_1 | MOTOR_4)
#define ELEVATOR_DOWN    (MOTOR_2 | MOTOR_3)
#define RUDDER_RIGHT     (MOTOR_4 | MOTOR_2)
#define RUDDER_LEFT      (MOTOR_1 | MOTOR_3)

#endif  /* PROTOCOL_SYMA_X5C_MODEL */

#ifdef PROTOCOL_SYMA_X5C_NEW_MODEL
#define PACKET_SIZE  16
#define PAYLOADSIZE 10      // was 10, receive data pipes set to this size, but unused
#define MAX_PACKET_SIZE 16   // X11,X12,X5C-1 10-byte, X5C 16-byte
#define RX_ADDR_SIZE  5
#elif defined(PROTOCOL_SYMA_X5C_MODEL)
#define FIRST_PACKET_SIZE 10  // PAYLOADSIZE should be the same as the packet size in order to receive the packet.
#define PACKET_SIZE  10
#define PAYLOADSIZE 10      // was 10, receive data pipes set to this size, but unused
#define MAX_PACKET_SIZE 16   // X11,X12,X5C-1 10-byte, X5C 16-byte
#define RX_ADDR_SIZE  5
#else
#error "No protocol defined !!"
#endif


extern const void *SYMAX_Cmds(enum ProtoCmds cmd);
