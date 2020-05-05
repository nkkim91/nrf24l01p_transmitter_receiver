#ifdef SYMAX_MODEL

/* nRF24L01P connection
 *  CE : D7
 *  CSN : D8
 *  MOSI : D11
 *  MISO : D12
 *  SCK  : D13
 */

#include "symax_nrf24l01.h"

static u8 packet[MAX_PACKET_SIZE];  // used for TX and RX address packet receiving. Rx data packet is received into respective buffer such as SymaxX5CRxAirData and gstEcompassRxAirPacketData

static u8 packet_size;
static u16 counter;
static u32 unTxPacketCounter;
static u8 throttle, rudder, elevator, aileron, ucFlags = 0;

#ifdef NRF24L01_TRANSMITTER_MODE
static u8 rx_tx_addr[RXTX_ADDR_SIZE];
#ifdef NRF24L01_TRANSMITTER_USE_FIXED_RXTX_ADDR
static u8 rx_tx_addr_fixed[RXTX_ADDR_SIZE] = { 0xBA, 0xEE, 0x0B, 0x00, 0xA2 }; // Symax X5C Transmitter of my own
//static u8 rx_tx_addr_fixed[RXTX_ADDR_SIZE] = { 0x7F, 0x49, 0xDC, 0x81, 0xA2 }; // Multiprotocol Transmitter
#endif
#endif /* NRF24L01_TRANSMITTER_MODE */

#ifdef NRF24L01_RECEIVER_MODE
//u8 ucRxTxAddr[sizeof(ucChansBind)][RX_ADDR_SIZE];
u8 ucLockedRxTxAddr[RX_ADDR_SIZE];
#endif /* PROTOCOL_SYMA_X5C_RECEIVER_MODE */

#ifdef SYMA_X5C_NEW_MODEL
// frequency channel management
#define MAX_RF_CHANNELS    17
#else
#define MAX_RF_CHANNELS    4
#endif

u8 ucCurrentChan;
static u8 chans[MAX_RF_CHANNELS];  /* use 4 out of 17 which is the max */
static u8 num_rf_channels;
static u8 phase;

enum {
    SYMAX_INIT1 = 0,
    SYMAX_BIND2,
    SYMAX_BIND3,
    SYMAX_DATA
};

// Bit vector from bit position
#define BV(bit) (1 << bit)

/* NK -    9    checksum (all bytes xor'd together then added to 0x55) */
static uint8_t __attribute__((unused)) SYMAX_checksum(u8 *data)
{
    u8 sum = data[0];

    for (int i=1; i < packet_size-1; i++)
#ifdef SYMA_X5C_NEW_MODEL
            sum += data[i];
#elif defined(PROTOCOL_SYMA_X5C_MODEL)
            sum ^= data[i];
#else
#error "No protocol defined !!"
#endif            

#ifdef SYMA_X5C_NEW_MODEL
    return sum + 0;
#elif defined(PROTOCOL_SYMA_X5C_MODEL)
    return sum + 0x55;
#else
#error "No protocol defined !!"
#endif
}

#define BABS(X) (((X) < 0) ? -(u8)(X) : (X))
// Channel values are sign + magnitude 8bit values


#ifdef PROTOCOL_SYMA_X5C_MODEL
//static volatile int32_t raw[NUM_SOURCES + 1];
static u32 rand_seed = 0xb2c54a2ful;
// Linear feedback shift register with 32-bit Xilinx polinomial x^32 + x^22 + x^2 + x + 1
static const uint32_t LFSR_FEEDBACK = 0x80200003ul;
static const uint32_t LFSR_INTAP = 32-1;
static void update_lfsr(uint32_t *lfsr, uint8_t b)
{
    for (int i = 0; i < 8; ++i) {
        *lfsr = (*lfsr >> 1) ^ ((-(*lfsr & 1u) & LFSR_FEEDBACK) ^ ~((uint32_t)(b & 1) << LFSR_INTAP));
        b >>= 1;
    }
}

u32 rand32_r(u32 *seed, u8 update)
{
    if(! seed)
        seed = &rand_seed;
    update_lfsr(seed, update);
    return *seed;
}

u32 rand32()
{
    return rand32_r(0, 0);
}
#endif  /* PROTOCOL_SYMA_X5C_MODEL */


#ifdef PROTOCOL_SYMA_X5C_TRANSMITTER_MODE
static void SYMAX_read_controls(u8* throttle, u8* rudder, u8* elevator, u8* aileron, u8* pucFlags)
{
  gnPosRX = analogRead(gnM2R_xPin); /* aileron/roll/mode2_right_x */
  gnPosRY = analogRead(gnM2R_yPin);  /* pitch/elevator/mode2_right_y */
  
#ifdef EXT_BUTTON
  gnPosRX += (gstRxPacketData.stTxData.stTrimData.cAileron * TRIM_AMPLFY_FACTOR);
  if( gnPosRX > ANALOG_INPUT_MAX ) {
    gnPosRX = ANALOG_INPUT_MAX;  
  }
  if( gnPosRX < ANALOG_INPUT_MIN ) {
    gnPosRX = ANALOG_INPUT_MIN;  
  }

  gnPosRY -= (gstRxPacketData.stTxData.stTrimData.cElevator * TRIM_AMPLFY_FACTOR);
  if( gnPosRY > ANALOG_INPUT_MAX ) {
    gnPosRY = ANALOG_INPUT_MAX;  
  }
  if( gnPosRY < ANALOG_INPUT_MIN ) {
    gnPosRY = ANALOG_INPUT_MIN;  
  }
#endif  // EXT_BUTTON
  
  gnPosRAdjX = map(gnPosRX, 0, 1023, 0, 255);
  gnPosRAdjY = map(gnPosRY, 0, 1023, 0, 255);

  gnPosRAdjX = 255 - gnPosRAdjX - 128;
  *aileron = ((gnPosRAdjX < 0 ? 0x80 : 0) | BABS(gnPosRAdjX));  /* aileron/roll/mode2_right_x */  
#if defined(CONFIG_SYMA_X5C_TRANSMITTER_ELEPHANT_NK) || defined(CONFIG_SYMA_X5C_TRANSMITTER_PLUS_NK)
  gnPosRAdjY = 255 - gnPosRAdjY - 128;  /* Forward (high), Backward (Low) */
#else
  gnPosRAdjY = gnPosRAdjY - 128;  /* Forward (high), Backward (Low) */
#endif
  *elevator = ((gnPosRAdjY < 0 ? 0x80 : 0) | BABS(gnPosRAdjY));  /* pitch/elevator/mode2_right_y */  
  
  gnPosLX = analogRead(gnM2L_xPin);
  gnPosLY = analogRead(gnM2L_yPin);
  
#ifdef EXT_BUTTON

  gnPosLX += (gstRxPacketData.stTxData.stTrimData.cRudder * TRIM_AMPLFY_FACTOR);
  if( gnPosLX > ANALOG_INPUT_MAX ) {
    gnPosLX = ANALOG_INPUT_MAX;  
  }
  if( gnPosLX < ANALOG_INPUT_MIN ) {
    gnPosLX = ANALOG_INPUT_MIN;  
  }
#endif  // EXT_BUTTON
  gnPosLAdjX = map(gnPosLX, 0, 1023, 0, 255);
  gnPosLAdjY = map(gnPosLY, 0, 1023, 0, 255);  /* looks like 127(0x7F) is the max. 0x80 and above seems not add more throttling power */

#if defined(CONFIG_SYMA_X5C_TRANSMITTER_ELEPHANT_NK) || defined(CONFIG_SYMA_X5C_TRANSMITTER_PLUS_NK)
  gnPosLAdjX = 255 - gnPosLAdjX - 128;
#else
  gnPosLAdjX = gnPosLAdjX - 128;  
#endif
  *rudder = ((gnPosLAdjX < 0 ? 0x80 : 0) | BABS(gnPosLAdjX));  /* yaw/rudder/mode2_left_x */

#if defined(CONFIG_SYMA_X5C_TRANSMITTER_ELEPHANT_NK) || defined(CONFIG_SYMA_X5C_TRANSMITTER_PLUS_NK)
  gnPosLAdjY = 255 - gnPosLAdjY;
#else
  gnPosLAdjY = gnPosLAdjY;            
#endif
  *throttle = (u8)gnPosLAdjY;              /* throttle/mode2_left_y */

#ifdef JOY_BUTTON

  key_state = 0x00;
  if( ReadKeyInput(&key_state) == 1 ) { /* Pressed */

    if( key_state & 0x01 ) {  /* CAMERA */
      *pucFlags |= FLAG_PICTURE;
    }
    else {
      *pucFlags &= ~FLAG_PICTURE;
    }

    if( key_state & 0x02 ) {  /* VIDEO */
      *pucFlags |= FLAG_VIDEO;
    }
    else {
      *pucFlags &= ~FLAG_VIDEO;
    }
  }
#elif defined(EXT_BUTTON)

  if ( gstRxPacketData.stTxData.unButtonFlag & BUTTON_FLAG_CAMERA_IDX )
  {
#ifdef RAINBOW_LED
    RainbowLED_DebugLight(RB_LED_POS_MSG2, RB_LED_MSG2_2);
#endif    
    *pucFlags |= FLAG_PICTURE;
  } else {
#ifdef RAINBOW_LED
    RainbowLED_DebugLight(RB_LED_POS_MSG2, RB_LED_MSG2_1);
#endif    
    *pucFlags &= ~FLAG_PICTURE;
  }

  if ( gstRxPacketData.stTxData.unButtonFlag & BUTTON_FLAG_VIDEO_IDX )
  {
#ifdef RAINBOW_LED
    RainbowLED_DebugLight(RB_LED_POS_MSG3, RB_LED_MSG3_2);
#endif    
    *pucFlags |= FLAG_VIDEO;
  } else {
#ifdef RAINBOW_LED
    RainbowLED_DebugLight(RB_LED_POS_MSG3, RB_LED_MSG3_1);
#endif    
    *pucFlags &= ~FLAG_VIDEO;
  }
#else  // EXT_BUTTON
  
#endif  // JOY_BUTTON

#if defined(NKD_SPOT)
  printf("%7lu) ## NK [%s:%d] (%02x:%02x) (%02x:%02x) (%02x)\n", millis(), __func__, __LINE__, (u8)*rudder, (u8)*throttle, (u8)*aileron, (u8)*elevator, (u8)*pucFlags);
  printf("%7lu) ## NK [%s:%d] gnPosLX/gnPosLAdjX = (%d:%d), gnPosLY/gnPosLAdjY = (%d:%d), gnPosRX/gnPosRAdjX = (%d:%d), gnPosRY/gnPosRAdjY = (%d:%d)\n", millis(), __func__, __LINE__, gnPosLX, gnPosLAdjX, gnPosLY, gnPosLAdjY, gnPosRX, gnPosRAdjX, gnPosRY, gnPosRAdjY);
#endif

  /* NK - helpful for tunning */
#if defined(NKD_DELTA)
  if( gnPosLAdjX_Prev != gnPosLAdjX || gnPosLAdjY_Prev != gnPosLAdjY || gnPosRAdjX_Prev != gnPosRAdjX || gnPosRAdjY_Prev != gnPosRAdjY || ucFlag_Prev != *pucFlags ) {
    printf("%7lu) ## NK [%s:%d] gnPosLX/gnPosLAdjX = (%d:%d), gnPosLY/gnPosLAdjY = (%d:%d), gnPosRX/gnPosRAdjX = (%d:%d), gnPosRY/gnPosRAdjY = (%d:%d) *pucFlag = 0x%02x\n", millis(), __func__, __LINE__, gnPosLX, gnPosLAdjX, gnPosLY, gnPosLAdjY, gnPosRX, gnPosRAdjX, gnPosRY, gnPosRAdjY, *pucFlags);

    gnPosLX_Prev = gnPosLX; gnPosLAdjX_Prev = gnPosLAdjX;
    gnPosLY_Prev = gnPosLY; gnPosLAdjY_Prev = gnPosLAdjY;
    gnPosRX_Prev = gnPosRX; gnPosRAdjX_Prev = gnPosRAdjX;
    gnPosRY_Prev = gnPosRY; gnPosRAdjY_Prev = gnPosRAdjY;
    ucFlag_Prev = *pucFlags;
  }
#endif
}
#endif /* PROTOCOL_SYMA_X5C_TRANSMITTER_MODE */

#if defined(PROTOCOL_SYMA_X5C_NEW_MODEL)

#define X5C_CHAN2TRIM(X) ((((X) & 0x80 ? 0xff - (X) : 0x80 + (X)) >> 2) + 0x20)
static void SYMAX_build_packet_x5c(u8 bind)
{
  if (bind) {
    memset(packet, 0, packet_size);
    packet[7] = 0xae;
    packet[8] = 0xa9;
    packet[14] = 0xc0;
    packet[15] = 0x17;
  } else {
    SYMAX_read_controls(&throttle, &rudder, &elevator, &aileron, &ucFlags);

    packet[0] = throttle;
    packet[1] = rudder;
    packet[2] = elevator ^ 0x80;  // reversed from default
    packet[3] = aileron;
    packet[4] = X5C_CHAN2TRIM(rudder ^ 0x80);     // drive trims for extra control range
    packet[5] = X5C_CHAN2TRIM(elevator);
    packet[6] = X5C_CHAN2TRIM(aileron ^ 0x80);
    packet[7] = 0xae;
    packet[8] = 0xa9;
    packet[9] = 0x00;
    packet[10] = 0x00;
    packet[11] = 0x00;
    packet[12] = 0x00;
    packet[13] = 0x00;
    packet[14] = (ucFlags & FLAG_VIDEO   ? 0x10 : 0x00) 
                   | (ucFlags & FLAG_PICTURE ? 0x08 : 0x00)
                   | (ucFlags & FLAG_FLIP    ? 0x01 : 0x00)
                   | 0x04;  // always high rates (bit 3 is rate control)
    packet[15] = SYMAX_checksum(packet);
  }
}
#elif defined(PROTOCOL_SYMA_X5C_MODEL)

#if defined(CONFIG_SYMA_X5C_TRANSMITTER_NK) || defined(CONFIG_SYMA_X5C_TRANSMITTER_ELEPHANT_NK) || defined(CONFIG_SYMA_X5C_TRANSMITTER_PLUS_NK)
static void SYMAX_build_packet(u8 bind) {
  if (bind) {  /* Transmit the RX/TX address */
    packet[0] = rx_tx_addr[4];
    packet[1] = rx_tx_addr[3];
    packet[2] = rx_tx_addr[2];
    packet[3] = rx_tx_addr[1];
    packet[4] = rx_tx_addr[0];
    packet[5] = 0xaa;
    packet[6] = 0xaa;
    packet[7] = 0xaa;
    packet[8] = 0x00;
  } else {
    SYMAX_read_controls(&throttle, &rudder, &elevator, &aileron, &ucFlags);

    packet[0] = throttle;
    packet[1] = elevator;
    packet[2] = rudder;
    packet[3] = aileron;
    packet[4] = (ucFlags & FLAG_VIDEO   ? 0x80 : 0x00) 
                  | (ucFlags & FLAG_PICTURE ? 0x40 : 0x00); // FLAG_VIDEO : 0x02, FLAG_PICTURE : 0x04
    // use trims to extend controls
    packet[5] = (elevator >> 2) | 0xc0;  // always high rates (bit 7 is rate control)
    packet[6] = (rudder >> 2)   | (ucFlags & FLAG_FLIP  ? 0x40 : 0x00);
    packet[7] = (aileron >> 2)  | (ucFlags & FLAG_HEADLESS ? 0x80 : 0x00);
    packet[8] = 0x00;
  }
  packet[9] = SYMAX_checksum(packet);
}

#elif defined(CONFIG_HMC5883L_GY273_TRANSMITTER_NK)
static void SYMAX_build_packet(u8 bind) {
  int i;
  
  if (bind) {  /* Transmit the RX/TX address */
    packet[0] = rx_tx_addr[4];
    packet[1] = rx_tx_addr[3];
    packet[2] = rx_tx_addr[2];
    packet[3] = rx_tx_addr[1];
    packet[4] = rx_tx_addr[0];
    packet[5] = 0xaa;
    packet[6] = 0xaa;
    packet[7] = 0xaa;
    packet[8] = 0x00;
    packet[9] = SYMAX_checksum(packet);    
  } else {

//    read_hmc5883l_data(&gstEcompassTxAirPacketData);
    for( i = 0; i < packet_size - 1; i++ ) {
      packet[i] = gstEcompassTxAirPacketData.ucByte[i];
    }
    packet[9] = SYMAX_checksum(packet);
  }
}
#endif /* CONFIG_HMC5883L_GY273_TRANSMITTER_NK */

#else /* PROTOCOL_SYMA_X5C_MODEL */
#error "No protocol defined !!"
#endif

#ifdef PROTOCOL_SYMA_X5C_TRANSMITTER_MODE

static void SYMAX_send_packet(u8 bind)
{
#if defined(NKD_FUNC)
  printf("%7lu) ## NK [%s:%d] [+]\n", millis(), __func__, __LINE__);
#endif

#if defined(NKD_DEBUG)
  printf("%7lu) ## NK [%s:%d] bind : %d, ucCurrentChan : %d\n", millis(), __func__, __LINE__, bind, ucCurrentChan);
#endif
  
#ifdef SYMA_X5C_NEW_MODEL
  SYMAX_build_packet_x5c(bind);
#elif defined(PROTOCOL_SYMA_X5C_MODEL)
  SYMAX_build_packet(bind); // 
#else
#error "No protocol defined !!"
#endif

  // NK - it's necessary to be here because the flags has not cleared/reset after sending a previous packet
  // clear packet status bits and TX FIFO
  NRF24L01_WriteReg(NRF24L01_07_STATUS, (1 << NRF24L01_07_RX_DR)    //reset the flag(s)
                                      | (1 << NRF24L01_07_TX_DS)
                                      | (1 << NRF24L01_07_MAX_RT));  /* 0x70 */

  // NK - Is it always necessary to be called every times ?
#if 0
  NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_MASK_TX_DS)   // switch to TX mode
                                      | (1 << NRF24L01_00_EN_CRC)    
                                      | (1 << NRF24L01_00_CRCO)
                                      | (1 << NRF24L01_00_PWR_UP));  /* 0x2E */
#else
  NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_MASK_RX_DR)
                                      | (1 << NRF24L01_00_MASK_TX_DS)   // switch to TX mode
                                      | (1 << NRF24L01_00_MASK_MAX_RT)                                      
                                      | (1 << NRF24L01_00_EN_CRC)    
                                      | (1 << NRF24L01_00_CRCO)
                                      | (1 << NRF24L01_00_PWR_UP));  /* 0x2E */

#endif


  NRF24L01_WriteReg(NRF24L01_05_RF_CH, chans[ucCurrentChan]);  /* 75, 48, 64, 32 */
  NRF24L01_FlushTx();

  NRF24L01_WritePayload(packet, packet_size);  /* packet_size = 10 */

#if defined(NKD_DEBUG)
  printf("%7lu) ## NK [%s:%d] pkt %ld: chans[%d] : %d, bind : %d, data : ", millis(), __func__, __LINE__, unTxPacketCounter, ucCurrentChan, chans[ucCurrentChan], bind);
  for(int i=0; i < packet_size; i++) {
    printf("%02x ", packet[i]);
  }
  printf("\n");
#endif

  if (unTxPacketCounter++ % 2) {   // use each channel twice
    ucCurrentChan = (ucCurrentChan + 1) % num_rf_channels;
  }

  // NK - Is it always necessary to be called every times ?
#ifdef PROTOCOL_SYMA_X5C_MODEL
  // Check and adjust transmission power. We do this after
  // transmission to not bother with timeout after power
  // settings change -  we have plenty of time until next
  // packet.
  NRF24L01_SetPower(TXPOWER_150mW);
#else
  NRF24L01_SetPower(TXPOWER_150mW);
#endif

#if 0 /* Can skip this because the flags are reset/cleared just before sending a new packet in this function */
  delayMicroseconds(1000);  // wait until TX_DS is set so to clear it

  // clear packet status bits and TX FIFO
  NRF24L01_WriteReg(NRF24L01_07_STATUS, (1 << NRF24L01_07_RX_DR)    //reset the flag(s)
                                      | (1 << NRF24L01_07_TX_DS)
                                      | (1 << NRF24L01_07_MAX_RT));  /* 0x70 */
#endif

#if defined(NKD_FUNC)
  printf("%7lu) ## NK [%s:%d] [-]\n", millis(), __func__, __LINE__);
#endif
}

#elif defined(PROTOCOL_SYMA_X5C_RECEIVER_MODE)

/* moved to symax_nrf24l01_rx.ino */

#endif /* PROTOCOL_SYMA_X5C_RECEIVER_MODE */

#ifdef PROTOCOL_SYMA_X5C_MODEL
void MCU_SerialNumber(u8 *var, int len)
{
#if 0
  if(Transmitter.txid) {
        int l = len > 16 ? 16 : len;
        u32 id[4];
        u32 seed = 0x4d3ab5d0ul;
        for(int i = 0; i < 4; i++)
            rand32_r(&seed, Transmitter.txid >> (8*i));
        for(int i = 0; i < 4; i++)
            id[i] = rand32_r(&seed, 0x00);
        memcpy(var, &id[0], l);
        return;
    }
#endif
    int l = len > 12 ? 12 : len;
    // Every STM32 should have 12 bytes long unique id at 0x1FFFF7E8
    const u8 *stm32id = (u8*) 0x1FFFF7E8;
    for(int i = 0; i < l; i++) {
        var[i] = *stm32id++;
    }
    while(l < len) {
        var[l++] = 0x00;
    }
}

#ifdef PROTOCOL_SYMA_X5C_TRANSMITTER_MODE
// Generate address to use from TX id and manufacturer id (STM32 unique id)
static void initialize_rx_tx_addr()
{
    u32 lfsr = 0xb2c54a2ful;
    int i;

#ifndef USE_FIXED_MFGID  /* NK : Not a emulator */
    u8 var[12];
    MCU_SerialNumber(var, 12);
#if defined(NKD_DEBUG)
    printf("%7lu) ## NK [%s:%d] Manufacturer id: ", millis(), __func__, __LINE__);
#endif
    for (int i = 0; i < 12; ++i) {
#if defined(NKD_DEBUG)
        printf("%02X", var[i]);
#endif
        rand32_r(&lfsr, var[i]);
    }
#if defined(NKD_DEBUG)
    printf("\n");
#endif
#endif

#if 0
    if (Model.fixed_id) {
       for (u8 i = 0, j = 0; i < sizeof(Model.fixed_id); ++i, j += 8)
           rand32_r(&lfsr, (Model.fixed_id >> j) & 0xff);
    }
#endif    
    // Pump zero bytes for LFSR to diverge more
    for (u8 i = 0; i < sizeof(lfsr); ++i) rand32_r(&lfsr, 0);

    rx_tx_addr[4] = 0xa2;  /* Any reason, part of the rx/tx address is fixed to 0xA2 ? */
    for (u8 i = 0; i < sizeof(rx_tx_addr)-1; ++i) {
        rx_tx_addr[i] = lfsr & 0xff;
        rand32_r(&lfsr, i);
    }

#if defined(NKD_DEBUG)
    printf("%7lu) ## NK [%s:%d] rx_tx_addr : ", millis(), __func__, __LINE__);
    for(i = 0; i < RXTX_ADDR_SIZE; i++) {
      printf("0x%02x ", rx_tx_addr[i]);
    }
    printf("\n");
#endif
}
#endif /* PROTOCOL_SYMA_X5C_TRANSMITTER_MODE */
#endif /* PROTOCOL_SYMA_X5C_MODEL */


static void symax_init0()
{
#ifdef PROTOCOL_SYMA_X5C_MODEL
  const u8 bind_rx_tx_addr[] = {0xab, 0xac, 0xad, 0xae, 0xaf}; // 1st Binder Address (0xAB, 0xAC, 0xAD, 0xAE, 0xAF)
#else
  const u8 rx_tx_addr_x5c[] = {0x6d,0x6a,0x73,0x73,0x73};   // X5C uses same address for bind and data
#endif
  u8 rx_tx_addr_clear[] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
  
  volatile uint8_t ucStatus;

#if defined(NKD_FUNC)
  printf("%7lu) ## NK [%s:%d] [+]\n", millis(), __func__, __LINE__);
#endif

  NRF24L01_Initialize();  /* rf_setup = 0x0F */

#ifdef NRF24L01_RECEIVER_MODE
  NRF24L01_SetTxRxMode(RX_EN);
#elif defined(NRF24L01_TRANSMITTER_MODE)
  NRF24L01_SetTxRxMode(TX_EN);
#endif

  NRF24L01_ReadReg(NRF24L01_07_STATUS);  /* Read NRF24L01 reg. */
  NRF24L01_WriteReg(NRF24L01_00_CONFIG, BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_CRCO)); 
  NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknoledgement
  NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x3F);  // Enable all data pipes (even though not used?)
  NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);   // 5-byte RX/TX address
  NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0xFF); // 4mS retransmit t/o, 15 tries (retries w/o AA?)
  NRF24L01_WriteReg(NRF24L01_05_RF_CH, FIRST_PACKET_RXTX_CHANNEL);  // 1st Channel (8)

#ifdef SYMA_X5C_NEW_MODEL
  NRF24L01_SetBitrate(NRF24L01_BR_1M);
  packet_size = PACKET_SIZE;  /* 16 */
#elif defined(PROTOCOL_SYMA_X5C_MODEL)
  NRF24L01_SetBitrate(NRF24L01_BR_250K);  /* NK */
  packet_size = PACKET_SIZE;  /* 10 */
#else
#error "No protocol defined !!"
#endif

  NRF24L01_SetPower(TXPOWER_150mW);  /* NK - Not full power ? */
  NRF24L01_WriteReg(NRF24L01_07_STATUS, (1 << NRF24L01_07_RX_DR)    // Clear data ready, data sent, and retransmit
                                      | (1 << NRF24L01_07_TX_DS)
                                      | (1 << NRF24L01_07_MAX_RT));
  NRF24L01_WriteReg(NRF24L01_08_OBSERVE_TX, 0x00);  // reset by writing NRF24L01_05_RF_CH
  NRF24L01_WriteReg(NRF24L01_09_CD, 0x00);
  NRF24L01_WriteReg(NRF24L01_0C_RX_ADDR_P2, 0xC3); // LSB byte of pipe 2 receive address
  NRF24L01_WriteReg(NRF24L01_0D_RX_ADDR_P3, 0xC4);
  NRF24L01_WriteReg(NRF24L01_0E_RX_ADDR_P4, 0xC5);
  NRF24L01_WriteReg(NRF24L01_0F_RX_ADDR_P5, 0xC6);
  NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, PAYLOADSIZE);   // bytes of data payload for pipe 1
  NRF24L01_WriteReg(NRF24L01_12_RX_PW_P1, PAYLOADSIZE);
  NRF24L01_WriteReg(NRF24L01_13_RX_PW_P2, PAYLOADSIZE);
  NRF24L01_WriteReg(NRF24L01_14_RX_PW_P3, PAYLOADSIZE);
  NRF24L01_WriteReg(NRF24L01_15_RX_PW_P4, PAYLOADSIZE);
  NRF24L01_WriteReg(NRF24L01_16_RX_PW_P5, PAYLOADSIZE);
  NRF24L01_WriteReg(NRF24L01_17_FIFO_STATUS, 0x00); // Just in case, no real bits to write here

#ifdef SYMA_X5C_NEW_MODEL

#ifdef NRF24L01_RECEIVER_MODE
  NRF24L01_WriteRegisterMulti(NRF24L01_0B_RX_ADDR_P1, rx_tx_addr_x5c, RXTX_ADDR_SIZE);
#else
  NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, rx_tx_addr_x5c, RXTX_ADDR_SIZE);
#endif

#else /* SYMA_X5C_NEW_MODEL */

#ifdef NRF24L01_RECEIVER_MODE
  NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, rx_tx_addr_clear, RXTX_ADDR_SIZE);
  NRF24L01_WriteRegisterMulti(NRF24L01_0B_RX_ADDR_P1, bind_rx_tx_addr, RXTX_ADDR_SIZE);  /* NK - 0xab,0xac,0xad,0xae,0xaf */
#else
  NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, bind_rx_tx_addr, RXTX_ADDR_SIZE);  /* NK - 0xab,0xac,0xad,0xae,0xaf */
  // For test
  NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, bind_rx_tx_addr, RXTX_ADDR_SIZE);
#endif

#endif // SYMA_X5C_NEW_MODEL

  NRF24L01_ReadReg(NRF24L01_07_STATUS);

  /************************************************************************************************************/
  // Check for Beken BK2421/BK2423 chip
  // It is done by using Beken specific activate code, 0x53
  // and checking that status register changed appropriately
  // There is no harm to run it on nRF24L01 because following
  // closing activate command changes state back even if it
  // does something on nRF24L01
  NRF24L01_Activate(0x53); // magic for BK2421 bank switch

#if defined(NKD_DEBUG)
  printf("%7lu) ## NK [%s:%d] Switching banks\n", millis(), __func__, __LINE__);
#endif
  
  if (NRF24L01_ReadReg(NRF24L01_07_STATUS) & 0x80) {  /* bit 7 = 0 (bank 0), bit 7 = 1 (bank 1) */
#if defined(NKD_DEBUG)
    printf("%7lu) ## NK [%s:%d] BK2421 detected\n", millis(), __func__, __LINE__);
#endif
    // Beken registers don't have such nice names, so we just mention
    // them by their numbers
    // It's all magic, eavesdropped from real transfer and not even from the
    // data sheet - it has slightly different values
    NRF24L01_WriteRegisterMulti(0x00, (u8 *) "\x40\x4B\x01\xE2", 4);
    NRF24L01_WriteRegisterMulti(0x01, (u8 *) "\xC0\x4B\x00\x00", 4);
    NRF24L01_WriteRegisterMulti(0x02, (u8 *) "\xD0\xFC\x8C\x02", 4);
    NRF24L01_WriteRegisterMulti(0x03, (u8 *) "\x99\x00\x39\x21", 4);  // 0x99003941 (?)
    NRF24L01_WriteRegisterMulti(0x04, (u8 *) "\xF9\x96\x82\x1B", 4);
    NRF24L01_WriteRegisterMulti(0x05, (u8 *) "\x24\x06\x7F\xA6", 4);
    NRF24L01_WriteRegisterMulti(0x06, (u8 *) "\x00\x00\x00\x00", 4);
    NRF24L01_WriteRegisterMulti(0x07, (u8 *) "\x00\x00\x00\x00", 4);
    NRF24L01_WriteRegisterMulti(0x08, (u8 *) "\x00\x00\x00\x00", 4);
    NRF24L01_WriteRegisterMulti(0x09, (u8 *) "\x00\x00\x00\x00", 4);
    NRF24L01_WriteRegisterMulti(0x0A, (u8 *) "\x00\x00\x00\x00", 4);
    NRF24L01_WriteRegisterMulti(0x0B, (u8 *) "\x00\x00\x00\x00", 4);
    NRF24L01_WriteRegisterMulti(0x0C, (u8 *) "\x00\x12\x73\x00", 4);
    NRF24L01_WriteRegisterMulti(0x0D, (u8 *) "\x46\xB4\x80\x00", 4);
    NRF24L01_WriteRegisterMulti(0x0E, (u8 *) "\x41\x10\x04\x82\x20\x08\x08\xF2\x7D\xEF\xFF", 11);
    NRF24L01_WriteRegisterMulti(0x04, (u8 *) "\xFF\x96\x82\x1B", 4);
    NRF24L01_WriteRegisterMulti(0x04, (u8 *) "\xF9\x96\x82\x1B", 4);
  } else {
#if defined(NKD_DEBUG)
    printf("%7lu) ## NK [%s:%d] nRF24L01 detected\n", millis(), __func__, __LINE__);
#endif
  }
  NRF24L01_Activate(0x53); // switch bank back
  /************************************************************************************************************/

#ifdef NRF24L01_RECEIVER_MODE
  NRF24L01_FlushRx();
#else
  NRF24L01_FlushTx();
#endif

  NRF24L01_ReadReg(NRF24L01_07_STATUS);
  
  /* NK - Not sure if it's necessary because the RX_P_NO bits are read-only. */
  NRF24L01_WriteReg(NRF24L01_07_STATUS, (NRF24L01_RX_P_EMPTY << NRF24L01_07_RX_P_NO));  /* 0x0e, bit(1~3) : RX_P_NO */
  /* NK - What is this for ? */
  NRF24L01_ReadReg(NRF24L01_00_CONFIG);
  
#ifdef NRF24L01_RECEIVER_MODE
  NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_EN_CRC)   // /* 0x0c, switch to RX mode */
                    | (1 << NRF24L01_00_CRCO)
                    | (1 << NRF24L01_00_PRIM_RX));
#if 0 /* was initial code */
  NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_EN_CRC)   // /* 0x0e, power on */
                    | (1 << NRF24L01_00_CRCO)
                    | (1 << NRF24L01_00_PWR_UP)
                    | (1 << NRF24L01_00_PRIM_RX));
#else
  NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_MASK_RX_DR)   // switch to RX mode (CRC0 - 2 bytes)
                    | (1 << NRF24L01_00_MASK_TX_DS)
                    | (1 << NRF24L01_00_MASK_MAX_RT)
                    | (1 << NRF24L01_00_EN_CRC)
                    | (1 << NRF24L01_00_CRCO)
                    | (1 << NRF24L01_00_PWR_UP)
                    | (1 << NRF24L01_00_PRIM_RX) );  /* 0x5F */
#endif
#else
  NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_EN_CRC)   // /* 0x0c, switch to TX mode */
                    | (1 << NRF24L01_00_CRCO));
#if 0
  NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_EN_CRC)   // /* 0x0e, power on */
                    | (1 << NRF24L01_00_CRCO)
                    | (1 << NRF24L01_00_PWR_UP));
#else
  NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_MASK_RX_DR)
                    | (1 << NRF24L01_00_MASK_TX_DS)
                    | (1 << NRF24L01_00_MASK_MAX_RT)
                    | (1 << NRF24L01_00_EN_CRC)   // /* 0x0e, power on */
                    | (1 << NRF24L01_00_CRCO)
                    | (1 << NRF24L01_00_PWR_UP));
#endif
                    
#endif

#if defined(NKD_FUNC)
  printf("%7lu) ## NK [%s:%d] [-]\n", millis(), __func__, __LINE__);
#endif
}

static void symax_init1()
{
  uint8_t i;
  volatile uint8_t ucStatus;

#ifdef NRF24L01_RECEIVER_MODE
  uint8_t ucPacketMatchFlag;
  uint32_t unFirstPacketTimeOut;
#endif

#if defined(NKD_FUNC)
  printf("%7lu) ## NK [%s:%d] [+]\n", millis(), __func__, __LINE__);
#endif

#ifdef NRF24L01_TRANSMITTER_MODE
  // duplicate stock tx sending strange packet (effect unknown)
  u8 first_packet[] = {0xf9, 0x96, 0x82, 0x1b, 0x20, 0x08, 0x08, 0xf2, 0x7d, 0xef, 0xff, 0x00, 0x00, 0x00, 0x00}; /* FIRST_PACKET_SIZE(15) bytes */

#ifdef PROTOCOL_SYMA_X5C_MODEL

#if defined(NRF24L01_TRANSMITTER_CHANNEL_SYMA_X5C_DEVICE)
  u8 chans_bind[] = {0x4b, 0x30, 0x40, 0x09}; // was { 0x4b, 0x30, 0x40, 0x20 }. but, changed from the actual air packet tranmitted by X5C board
#elif defined(NRF24L01_TRANSMITTER_CHANNEL_IN_SYMA_X5C_SOURCE_CODE)
  u8 chans_bind[] = {0x4b, 0x30, 0x40, 0x20};
#elif defined(NRF24L01_TRANSMITTER_CHANNEL_EXPERIMENT)
  u8 chans_bind[] = {0x1c, 0x1c, 0x1c, 0x1c};
#endif
  u8 chans_bind_x5c[] = {0x27, 0x1b, 0x39, 0x28, 0x24, 0x22, 0x2e, 0x36,
                           0x19, 0x21, 0x29, 0x14, 0x1e, 0x12, 0x2d, 0x18};
#endif

#elif defined(NRF24L01_RECEIVER_MODE)
  /***************************************************
   *
   * CH 32 (0x20) is the channel info. in the deviation source code. But, no packet is received on the air.
   * From the polling each CHs, found CH9 is the last one actually working.
   * ex) u8 ucChansBind[] = {0x4b, 0x30, 0x40, 0x20};  
   *
   ***************************************************/
#if defined(NRF24L01_RECEIVER_CHANNEL_SYMA_X5C_DEVICE)
  u8 ucChansBind[] = {0x4b, 0x30, 0x40, 0x09};  // Symax X5c device uses the channels { 0x4b, 0x30, 0x40, 0x09 }
#elif defined(NRF24L01_RECEIVER_CHANNEL_IN_SYMA_X5C_SOURCE_CODE)
  u8 ucChansBind[] = {0x4b, 0x30, 0x40, 0x20};  // whereas Multiprotocol4in1 and other syma protocol uses the channels { 0x4b, 0x30, 0x40, 0x20 }
#elif defined(NRF24L01_RECEIVER_CHANNEL_EXPERIMENT)
  u8 ucChansBind[] = {0x1c, 0x1c, 0x1c, 0x1c};
#endif

  u8 first_packet_expected[] = {0xf9, 0x96, 0x82, 0x1b, 0x20, 0x08, 0x08, 0xf2, 0x7d, 0xef, 0xff, 0x00, 0x00, 0x00, 0x00}; /* FIRST_PACKET_SIZE(15) bytes */
  u8 first_packet_received[MAX_PACKET_SIZE] = { 0x00 };
/****
 * bind address is defined as global variables
 **************/
#else  
#error "## NK - No NRF24L01 Mode defined !!"
#endif

#ifdef NRF24L01_TRANSMITTER_MODE
  NRF24L01_FlushTx();
#elif defined(NRF24L01_RECEIVER_MODE)
  NRF24L01_FlushRx();
  NRF24L01_FlushTx();
#else
#error "## NK - No NRF24L01 Mode defined !!"
#endif

  NRF24L01_WriteReg(NRF24L01_05_RF_CH, FIRST_PACKET_RXTX_CHANNEL);  /* CH : 8, write again (?) */

#ifdef NRF24L01_TRANSMITTER_MODE
    printf("%7lu) ## NK [%s:%d] Send first Pkt !!\n", millis(), __func__, __LINE__);
    NRF24L01_WritePayload(first_packet, FIRST_PACKET_SIZE);
#elif defined(NRF24L01_RECEIVER_MODE)

  // Receive first packet
  unFirstPacketTimeOut = millis() + FIRST_PACKET_TIMEOUT; /* 500 ms */

  for( ucPacketMatchFlag = 0; 1 ; ) {

      ucStatus = NRF24L01_ReadReg(NRF24L01_07_STATUS);

//      printf("%7lu) ## NK [%s:%d] ucStatus : 0x%02x\n", millis(), __func__, __LINE__, ucStatus);
          
      if( (long)(millis() - unFirstPacketTimeOut) >= 0 ) {  /* Not expired yet */
        printf("%7lu) ## NK [%s:%d] First packet - Time out !!\n", millis(), __func__, __LINE__);
        break;  // Timeout
      } else {
      }
  
      if ( ucStatus & (1 << NRF24L01_07_RX_DR) && (((ucStatus >> NRF24L01_07_RX_P_NO) & 0x07) >= NRF24L01_RX_P_0 && ((ucStatus >> NRF24L01_07_RX_P_NO) & 0x07) <= NRF24L01_RX_P_5)) {

#if defined(NKD_DEBUG_VERBOSE)
        printf("%7lu) ## NK [%s:%d] ucStatus : 0x%02x\n", millis(), __func__, __LINE__, ucStatus);
#endif

        NRF24L01_WriteReg(NRF24L01_07_STATUS, (1 << NRF24L01_07_RX_DR) );    // reset the flag(s) - Data Ready RX FIFO interrupt 
  
        if ( ucStatus & (1 << NRF24L01_07_TX_DS) ) {
          NRF24L01_WriteReg(NRF24L01_07_STATUS, (1 << NRF24L01_07_TX_DS) );    // reset the flag(s) - Data Sent TX FIFO interrupt
        }

        memset(first_packet_received, 0, FIRST_PACKET_SIZE);
        NRF24L01_ReadPayload(first_packet_received, FIRST_PACKET_SIZE);  // first packet size : 10 or 15
   
        // Check if first_packet is received correctly
        for(i = 0, ucPacketMatchFlag = 1; i < FIRST_PACKET_SIZE; i++ ) {
          if( first_packet_received[i] != first_packet_expected[i] ) {
            ucPacketMatchFlag = 0;
            printf("%7lu) ## NK [%s:%d] Received packet is different from expected !!\n", millis(), __func__, __LINE__);
            break;    /* Expect the exact data packet is received in the FIFO */
          }
        }
          
        if( ucPacketMatchFlag == 1 ) {  /* first packet is valid */
          printf("%7lu) ## NK [%s:%d] First packet is valid !!\n", millis(), __func__, __LINE__);
          break;
        } else {
          printf("%7lu) ## NK [%s:%d] \t", millis(), __func__, __LINE__);
          for(i = 0; i < FIRST_PACKET_SIZE; i++) {
            printf("0x%02x ", first_packet_received[i]);
          }
          printf("\n");
        }
      }
//      delay(1);
  }

  if( ucPacketMatchFlag == 0 ) {
#ifdef MELODY_NK
    MUSIC_Play(1); delay(200);
    MUSIC_Play(1); delay(200);
    MUSIC_Play(1); delay(200);    
#endif
#if defined(NKD_DEBUG)
    printf("%7lu) ## NK [%s:%d] Received Pkt different from expected !!\n", millis(), __func__, __LINE__);

    printf("%7lu) ## NK [%s:%d] \t", millis(), __func__, __LINE__);
    for(i = 0; i < FIRST_PACKET_SIZE; i++ ) {
      printf("0x%02x ", first_packet_received[i]);
    }
    printf("\n");
#endif
  } else {
#ifdef MELODY_NK
    MUSIC_Play(2);
#endif    
#if defined(NKD_DEBUG)
    printf("%7lu) ## NK [%s:%d] Received first valid Pkt !!\n", millis(), __func__, __LINE__);
    printf("## NK [%s:%d] \t", millis(), __func__, __LINE__);
    for(i = 0; i < FIRST_PACKET_SIZE; i++ ) {
      printf("0x%02x ", first_packet_received[i]);
    }
    printf("\n");
#endif
  }
#else /* NRF24L01_RECEIVER_MODE */
#error "## NK - No NRF24L01 Mode defined !!"
#endif

#ifdef SYMA_X5C_NEW_MODEL
  num_rf_channels = sizeof(chans_bind_x5c);  /* 16 bytes, {0x27, 0x1b, 0x39, 0x28, 0x24, 0x22, 0x2e, 0x36, 0x19, 0x21, 0x29, 0x14, 0x1e, 0x12, 0x2d, 0x18} */
  memcpy(chans, chans_bind_x5c, num_rf_channels);
#if defined(NKD_DEBUG)
  printf("%7lu) ## NK [%s:%d] ", millis(), __func__, __LINE__);
  for( i = 0; i < num_rf_channels; i++ ) {
    printf("0x%02x ", chans[i]);
  }  
  printf("\n");
#endif
#elif defined(PROTOCOL_SYMA_X5C_MODEL)

#ifdef NRF24L01_TRANSMITTER_MODE
  initialize_rx_tx_addr();   // Trasmitter generates the RX/TX Address used btw. trasmitter and receiver

#ifdef NRF24L01_TRANSMITTER_USE_FIXED_RXTX_ADDR
  memcpy(rx_tx_addr, rx_tx_addr_fixed, RXTX_ADDR_SIZE);
#endif

#if defined(NKD_INFO)
    printf("%7lu) ## NK [%s:%d] RX/TX Address : ", millis(), __func__, __LINE__);
    for(i = 0; i < RXTX_ADDR_SIZE; i++ ) { // Constant value needs to be updated.
       printf("0x%02x ", rx_tx_addr[i]); 
    }
    printf("\n");
#endif

  num_rf_channels = sizeof(chans_bind); /* {0x4b, 0x30, 0x40, 0x09} */
  memcpy(chans, chans_bind, num_rf_channels);
#elif defined(NRF24L01_RECEIVER_MODE)

  num_rf_channels = sizeof(ucChansBind);  /* 4 bytes, {0x4b, 0x30, 0x40, 0x09} */
  memcpy(chans, ucChansBind, num_rf_channels);

#endif // NRF24L01_RECEIVER_MODE

#else /* PROTOCOL_SYMA_X5C_MODEL */
#error "## NK - Protocol is not defined !!"
#endif

  ucCurrentChan = 0;  /* NK */
  unTxPacketCounter = 0;

#if defined(NKD_FUNC)
  printf("%7lu) ## NK [%s:%d] [-]\n", millis(), __func__, __LINE__);
#endif
}

// channels determined by last byte of tx address
static void SYMAX_set_channels(u8 address) {
  static const u8 start_chans_1[] = {0x0a, 0x1a, 0x2a, 0x3a};  /* 10, 26, 42, 58 */
  static const u8 start_chans_2[] = {0x2a, 0x0a, 0x42, 0x22};  /* 42, 10, 66, 34 */
  static const u8 start_chans_3[] = {0x1a, 0x3a, 0x12, 0x32};  /* 26, 58, 18, 50 */

  u8 laddress = address & 0x1f;
  u8 i;
  u32 *pchans = (u32 *)chans;   // avoid compiler warning

#if defined(NKD_FUNC)
  printf("%7lu) ## NK [%s:%d] [+]\n", millis(), __func__, __LINE__);
#endif

#ifdef NRF24L01_TRANSMITTER_MODE
  num_rf_channels = 4;
#elif defined(NRF24L01_RECEIVER_MODE)
//  num_rf_channels = 4;
#endif

  if (laddress < 0x10) {        /* 0 ~ 15 (16) */
    if (laddress == 6) laddress = 7;
    for(i=0; i < num_rf_channels; i++) {
      chans[i] = start_chans_1[i] + laddress;
    }
  } else if (laddress < 0x18) {  /* 16 ~ 23 (8) */
    for(i=0; i < num_rf_channels; i++) {
      chans[i] = start_chans_2[i] + (laddress & 0x07);
    }
    if (laddress == 0x16) {
      chans[0] += 1;
      chans[1] += 1;
    }
  } else if (laddress < 0x1e) {  /* 24 ~ 29 (6) */
    for(i=0; i < num_rf_channels; i++) {
      chans[i] = start_chans_3[i] + (laddress & 0x07);
    }
  } else if (laddress == 0x1e) {  /* 30 (1) */
      *pchans = 0x38184121;
  } else {
      *pchans = 0x39194121;
  }
  
#if defined(NKD_DEBUG_VERBOSE)	// Duplicated message, the other one from SYMAX_init2()
  printf("%7lu) ## NK [%s:%d] New RF Ch. : ", millis(), __func__, __LINE__);
  for(i = 0; i < num_rf_channels; i++) {
    printf("0x%02x ", chans[i]);
  }
  printf("\n");
#endif
  
#if defined(NKD_FUNC)
  printf("%7lu) ## NK [%s:%d] [-]\n", millis(), __func__, __LINE__);
#endif  
}

static void symax_init2()
{
#ifdef SYMA_X5C_NEW_MODEL
  u8 chans_data_x5c[] = {0x1d, 0x2f, 0x26, 0x3d, 0x15, 0x2b, 0x25, 0x24,
                           0x27, 0x2c, 0x1c, 0x3e, 0x39, 0x2d, 0x22};
#endif
  int i;

#if defined(NKD_FUNC)
  printf("%7lu) ## NK [%s:%d] [+]\n", millis(), __func__, __LINE__);
#endif

#ifdef SYMA_X5C_NEW_MODEL
  num_rf_channels = sizeof(chans_data_x5c);
  memcpy(chans, chans_data_x5c, num_rf_channels);
#elif defined(PROTOCOL_SYMA_X5C_MODEL)
#ifdef NRF24L01_TRANSMITTER_MODE

#if defined(NKD_DEBUG)
  printf("%7lu) ## NK [%s:%d] Init. RF Ch. : ", millis(), __func__, __LINE__);
  for(i = 0; i < num_rf_channels; i++) {
    printf("0x%02x ", chans[i]);
  }
  printf("\n");
#endif

#ifndef NRF24L01_USE_FIXED_CHANNELS
  SYMAX_set_channels(rx_tx_addr[0]);
#endif
#if defined(NKD_INFO)
  printf("%7lu) ## NK [%s:%d] New RF Ch. : ", millis(), __func__, __LINE__);
  for(i = 0; i < num_rf_channels; i++) {
    printf("0x%02x ", chans[i]);
  }
  printf("\n");
#endif

  printf("%7lu) ## NK [%s:%d] rx_tx_addr : ", millis(), __func__, __LINE__);
  for(i = 0; i < RXTX_ADDR_SIZE; i++) {
    printf("0x%02x ", rx_tx_addr[i]);
  }
  printf("\n");

  NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, rx_tx_addr, RXTX_ADDR_SIZE);
  // For Test
  NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, rx_tx_addr, RXTX_ADDR_SIZE);

#elif defined(NRF24L01_RECEIVER_MODE)

#if defined(NKD_DEBUG)
  printf("%7lu) ## NK [%s:%d] Init. RF Ch. : ", millis(), __func__, __LINE__);
  for(i = 0; i < num_rf_channels; i++) {
    printf("0x%02x ", chans[i]);
  }
  printf("\n");
#endif

#ifndef NRF24L01_USE_FIXED_CHANNELS
  SYMAX_set_channels(ucLockedRxTxAddr[0]);
#endif

#if defined(NKD_DEBUG)
  printf("%7lu) ## NK [%s:%d] New RF Ch.   : ", millis(), __func__, __LINE__);
  for(i = 0; i < num_rf_channels; i++) {
    printf("0x%02x ", chans[i]);
  }
  printf("\n");

  printf("%7lu) ## NK [%s:%d] ucLockedRxTxAddr : ", millis(), __func__, __LINE__);
  for(i = 0; i < RXTX_ADDR_SIZE; i++) {
    printf("0x%02x ", ucLockedRxTxAddr[i]);
  }
  printf("\n");
#endif
  
  NRF24L01_WriteRegisterMulti(NRF24L01_0B_RX_ADDR_P1, ucLockedRxTxAddr, RXTX_ADDR_SIZE);  /* NK - 0xab,0xac,0xad,0xae,0xaf */  
#else /* NRF24L01_RECEIVER_MODE */
#error "## NK - No NRF24L01 Mode defined !!"
#endif

#else /* PROTOCOL_SYMA_X5C_MODEL */
#error "## NK - No protocol defined !!"
#endif
  ucCurrentChan = 0;
  unTxPacketCounter = 0;

#if defined(NKD_FUNC)
  printf("%7lu) ## NK [%s:%d] [-]\n", millis(), __func__, __LINE__);
#endif  
  
}

#ifdef PROTOCOL_SYMA_X5C_RECEIVER_MODE

/* Moved to symax_nrf24l01_rx.ino */

#elif defined(PROTOCOL_SYMA_X5C_TRANSMITTER_MODE)

#if 1
/**************************************************************
 * 
 * As per the measurement through the DebugPin pulse, the symax_callback takes as follow.
 * 
 * CONFIG_SYMA_X5C_TRANSMITTER_PLUS_NK : 25 ms (with EXT_BUTTON enabled), 4.8 ms (without EXT_BUTTON)
 * CONFIG_HMC5883L_GY273_TRANSMITTER_NK : 4.4 ms
 * 
 *************************************************************/ 
static void symax_callback()
{
  unsigned char ucObserveTx;
  
#if defined(NKD_FUNC)
  printf("%7lu) ## NK [%s:%d] [-]\n", millis(), __func__, __LINE__);
#endif

#if defined(NKD_DEBUG)
    printf("%7lu) ## NK [%s:%d] phase : %d\n", millis(), __func__, __LINE__, phase);
#endif

#ifdef NK_DEBUG_LED
  digitalWrite(gnDebugLEDPin, 1);
#endif

  Timer1.stop();
  
  switch (phase) {
    case SYMAX_INIT1:
      symax_init1();
      phase = SYMAX_BIND2;
//      return FIRST_PACKET_DELAY;
#if defined(NKD_DEBUG)
      printf("%7lu) ## NK [%s:%d]\n", millis(), __func__, __LINE__);
#endif
      Timer1.initialize(FIRST_PACKET_DELAY);  /* 12000 us */
      Timer1.attachInterrupt( symax_callback );

#ifdef NK_DEBUG_LED
      digitalWrite(gnDebugLEDPin, 0);
#endif
      return;

    case SYMAX_BIND2:
      counter = BIND_COUNT;  /* NK, 345 * 500 us = 172 ms(?) */
      phase = SYMAX_BIND3;
      SYMAX_send_packet(1);
      break;

    case SYMAX_BIND3:
      if (counter == 0) {  /* NK, after 172 ms elapsed */
        symax_init2();
        phase = SYMAX_DATA;
        PROTOCOL_SetBindState(0);
#ifdef MELODY_NK
//        MUSIC_Play(MELODY_SUCCESS);  /* NK, seems tone library conflicts with the timer. so, the tone is not played */
#endif        
#ifdef NK_DEBUG_TIMER_MONITOR
      time_data_pkt_at = millis();
#endif
      } else {
        SYMAX_send_packet(1);
        counter -= 1;
      }
      break;


    case SYMAX_DATA:
      SYMAX_send_packet(0);
#ifdef NK_DEBUG_TIMER_MONITOR
      time_data_pkt++;
      if( !(time_data_pkt % NK_DEBUG_TIMER_MONITOR_PERIOD ) ) {   /* 4ms * #pks = xxx sec */
        printf("%7lu) ## NK [%s:%d] %lu ~ %lu : %lu Pkt xmit/recvd !!\n", millis(), __func__, __LINE__, time_data_pkt_at, millis(), time_data_pkt);
        ucObserveTx = NRF24L01_ReadReg(NRF24L01_08_OBSERVE_TX);
        if( ucObserveTx ) { // print on any counts (PLOS or ARC)
          NRF24L01_RegSummary();
        }

        if( (ucObserveTx & (0x0F << NRF24L01_08_ARC_CNT)) == (0x0F << NRF24L01_08_ARC_CNT) ) {
          CE_lo();
          delay(5);
          CE_hi();
          NRF24L01_FlushTx();
        }
        time_data_pkt_at = millis();
        time_data_pkt = 0;
      }
#endif
      break;
    }


#ifdef NK_DEBUG_LED
  digitalWrite(gnDebugLEDPin, 0);
#endif

  /* NK : the next callback is updated in the caller function in the deviationtx's code. but, it was modified to be done here before returns */    
  Timer1.initialize(PACKET_PERIOD);  /* 4000 us (4 ms) */
  Timer1.attachInterrupt( symax_callback );    

#if defined(NKD_FUNC)
  printf("%7lu) ## NK [%s:%d] [-]\n", millis(), __func__, __LINE__);
#endif
  return;
}
#else /* if 0 */

unsigned long nk_count=0;

static void symax_callback()
{
#ifdef NK_DEBUG_LED
  digitalWrite(gnDebugLEDPin, 1);
#endif

  Timer1.stop();

  Serial.println(millis());

#ifdef NK_DEBUG_LED
  digitalWrite(gnDebugLEDPin, 0);
#endif

#if 1
  Timer1.initialize(2000);  /* 4000 us (4 ms) */
  Timer1.attachInterrupt( symax_callback );    
#else
  Timer1.restart();
#endif

}
#endif /* if 0 */

#else
#error "## NK - No protocol RX/TX mode defined"
#endif

static uint8_t ucInitDone = 0;

static void initialize()
{
#if defined(NKD_FUNC)
  printf("%7lu) ## NK [%s:%d] [+]\n", millis(), __func__, __LINE__);
#endif

  if( !ucInitDone ) {

    unTxPacketCounter = 0;
    ucFlags = 0;

    symax_init0();                 /* No blocking code - Just initialize nRF24L01 registers */
    phase = SYMAX_INIT1;

    PROTOCOL_SetBindState(BIND_COUNT * PACKET_PERIOD / 1000);  /* NK, current time + (345 * 4000 usec / 1000 = 1380 ms) , set 'bind_time', main() -> EventLoop() -> PROTOCOL_CheckDialogs() -> PROTO_BINDING() */

    ucInitDone = 1;
  }

#ifdef PROTOCOL_SYMA_X5C_TRANSMITTER_MODE
  Timer1.initialize(INITIAL_WAIT);  /* NK - deviationtx => CLOCK_StartTimer(INITIAL_WAIT, symax_callback); 500 us */
  Timer1.attachInterrupt( symax_callback );
#elif defined(PROTOCOL_SYMA_X5C_RECEIVER_MODE)
  delayMicroseconds(INITIAL_WAIT);
  symax_receiver_loop();
#else
#error "## NK - No protocol RX/TX mode defined"
#endif

#if defined(NKD_FUNC)
  printf("%7lu) ## NK [%s:%d] [-]\n", millis(), __func__, __LINE__);
#endif
}

const void *SYMAX_Cmds(enum ProtoCmds cmd)
{
#if defined(NKD_FUNC)
  printf("%7lu) ## NK [%s:%d] [+]\n", millis(), __func__, __LINE__);
#endif

  switch(cmd) {
    case PROTOCMD_INIT:  initialize(); return 0;  /* Timer callback is set up */
    case PROTOCMD_DEINIT:
    case PROTOCMD_RESET:
#ifdef PROTOCOL_SYMA_X5C_TRANSMITTER_MODE
      Timer1.stop();  /* CLOCK_StopTimer(); */
#endif            

      return (void *)(NRF24L01_Reset() ? 1L : -1L);
    case PROTOCMD_CHECK_AUTOBIND: return (void *)1L; // always Autobind
    case PROTOCMD_BIND:  initialize(); return 0;
    case PROTOCMD_NUMCHAN: return (void *) 9L;
    case PROTOCMD_DEFAULT_NUMCHAN: return (void *)6L;
#if 0
    case PROTOCMD_CURRENT_ID: return Model.fixed_id ? (void *)((unsigned long)Model.fixed_id) : 0;
    case PROTOCMD_GETOPTIONS: return symax_opts;
    case PROTOCMD_TELEMETRYSTATE: return (void *)(long)PROTO_TELEM_UNSUPPORTED;
#endif
    default: break;
  }

#if defined(NKD_FUNC)
  printf("%7lu) ## NK [%s:%d] [-] \n", millis(), __func__, __LINE__);
#endif
  return 0;
}

#endif /* SYMAX_MODEL */
