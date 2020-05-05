#include "config.h"

#ifdef SERIAL_INPUT

#include "printf.h"

#include "nrf24l01p_transmitter.h"
//#include "neopixel_ws2812.h"
#include "serial_input.h"

#include "TX_Def.h"

/***************************************************
 *
 * Debug MASK Data structure / Macro       
 *
 ***************************************************/

extern union ChannelPacketData_b gstTxChannelPacketData, gstRxChannelPacketData, gstTempRxChannelPacketData;
extern eERR_CH_PKT_DATA_t geErrChannelPktData;
extern Adafruit_NeoPixel pixels;


extern void RainbowLED_DebugLight(uint8_t ucLEDPos, uint8_t ucMsgId);

static uint32_t stat_pkt_valid = 0;
static uint32_t stat_pkt_invalid = 0;
static uint32_t pre_stat_pkt_valid = 0;
static uint32_t pre_stat_pkt_invalid = 0;
static uint32_t stat_pkt_timeout = 0;
static uint32_t pre_stat_pkt_timeout = 0;
static uint32_t stat_pkt_crc_error = 0;
static uint32_t pre_stat_pkt_crc_error = 0;
static uint32_t stat_pkt_minmax_error = 0;
static uint32_t pre_stat_pkt_minmax_error = 0;
static uint32_t stat_pkt_magic_error = 0;
static uint32_t pre_stat_pkt_magic_error = 0;

static uint32_t pre_pkt_number = 0;


extern void RainbowLED_DebugLight(uint8_t ucLEDPos, uint8_t ucMsgId);

/*
 * gstTempRxChannelPacketData -> gstRxChannelPacketData and Channel_data
 * 
 * 
 */
void V2X2_read_control_serial() {

  int i, j, k;
  // put your main code here, to run repeatedly:

#if defined(NKD_FUNC)
  printf("%7lu) ## NK [%s:%d] [+] \n", millis(), __func__, __LINE__);
#endif

  geErrChannelPktData = RX_ChannelPktData();
#if defined(NKD_DEBUG)
  printf("## NK [%s:%d] geErrChannelPktData : %d\n", millis(), __func__, __LINE__, geErrChannelPktData);
#endif

#if 0
//  for(k = 0; k < 255; k+=10) {    
//  for(j = 0; j < 255; j+=10) {
  for(i = 0; i < 255; i+=10) {
      pixels.setPixelColor(0, pixels.Color(255,0,0));
      pixels.show();
      delay(500);
      pixels.setPixelColor(0, pixels.Color(0,0,0));
      pixels.show();
      delay(500);
      
//  }}
  }
  delay(5000);
#endif

  if( geErrChannelPktData >= ERR_CH_PKT_DATA_VALID ) {
    stat_pkt_valid++;
    gstRxChannelPacketData = gstTempRxChannelPacketData; // Copy the valid data to the static variable.
#if defined(NKD_DEBUG)
      printf("%7lu) ## NK [%s:%d] AETR : %u/%u/%u/%u ", millis(), __func__, __FUNC__, gstRxChannelPacketData.stChannelPacketData.stChannelData.ucAileron,
                                  gstRxChannelPacketData.stChannelPacketData.stChannelData.ucElevator,
                                  gstRxChannelPacketData.stChannelPacketData.stChannelData.ucThrottle,
                                  gstRxChannelPacketData.stChannelPacketData.stChannelData.ucRudder);
      for( i = 0; i < EXTENDED_CHANNELS; i++) {
        printf("- %u \n", gstRxChannelPacketData.stChannelPacketData.stChannelData.ucCH[i]);
      }
#endif
    if( gstRxChannelPacketData.stChannelPacketData.stChannelData.unPktNumber != pre_pkt_number ) {
#if defined(NKD_DEBUG)
        printf("%7lu) ## NK [%s:%d] ## PKT Lost !!\n", millis(), __func__, __LINE__);
#endif
      pre_pkt_number = gstRxChannelPacketData.stChannelPacketData.stChannelData.unPktNumber;
    }

    Channel_data[AILERON]   = gstRxChannelPacketData.stChannelPacketData.stChannelData.ucAileron;
    Channel_data[ELEVATOR]  = gstRxChannelPacketData.stChannelPacketData.stChannelData.ucElevator;
    Channel_data[THROTTLE]  = gstRxChannelPacketData.stChannelPacketData.stChannelData.ucThrottle;
    Channel_data[RUDDER]    = gstRxChannelPacketData.stChannelPacketData.stChannelData.ucRudder;

  } else {
    stat_pkt_invalid++;
    
    switch( geErrChannelPktData ) {
      case ERR_CH_PKT_DATA_TIMEOUT :
        stat_pkt_timeout++;      
        break;
      case ERR_CH_PKT_DATA_CRC_ERROR :
        stat_pkt_crc_error++;
        break;
      case ERR_CH_PKT_DATA_MINMAX_ERROR :
        stat_pkt_minmax_error++;                  
        break;
      case ERR_CH_PKT_DATA_MAGIC_ERROR:
        stat_pkt_magic_error++;
        break;
      default :
        break;
    }
  }

  if( stat_pkt_valid != pre_stat_pkt_valid ) {
    printf("%7lu) ## NK [%s:%d] Valid : %lu, PreValid : %lu\n", millis(), __func__, __LINE__, stat_pkt_valid, pre_stat_pkt_valid);
    printf("%7lu) ## NK [%s:%d] Invalid : %lu\n", millis(), __func__, __LINE__, stat_pkt_invalid);
    pre_stat_pkt_valid = stat_pkt_valid;
  }

  if( stat_pkt_invalid != pre_stat_pkt_invalid ) {
//    printf("%7lu) ## NK [%s:%d] InValid : %u, PreInvalid : %u\n", millis(), __func__, __LINE__, stat_pkt_invalid, pre_stat_pkt_invalid);
    pre_stat_pkt_invalid = stat_pkt_invalid;
  }

  if( stat_pkt_timeout != pre_stat_pkt_timeout ) {
//    printf("%7lu) ## NK [%s:%d] TimeOut : %u, PreTimeOut : %u\n", millis(), __func__, __LINE__, stat_pkt_timeout, pre_stat_pkt_timeout);
    pre_stat_pkt_timeout = stat_pkt_timeout;
  }

  if( stat_pkt_crc_error != pre_stat_pkt_crc_error ) {
    printf("%7lu) ## NK [%s:%d] CRCError : %lu, PreCRCError : %lu\n", millis(), __func__, __LINE__, stat_pkt_crc_error, pre_stat_pkt_crc_error);
    pre_stat_pkt_crc_error = stat_pkt_crc_error;
  }

  if( stat_pkt_minmax_error != pre_stat_pkt_minmax_error ) {
    printf("%7lu) ## NK [%s:%d] MinMaxError : %lu, PreMinMaxError : %lu\n", millis(), __func__, __LINE__, stat_pkt_minmax_error, pre_stat_pkt_minmax_error);
    pre_stat_pkt_minmax_error = stat_pkt_minmax_error;
  }

  if( stat_pkt_magic_error != pre_stat_pkt_magic_error ) {
    printf("%7lu) ## NK [%s:%d] MagicError : %lu, PreMagicError : %lu\n", millis(), __func__, __LINE__, stat_pkt_magic_error, pre_stat_pkt_magic_error);
    pre_stat_pkt_magic_error = stat_pkt_magic_error;
  }

#if defined(NKD_FUNC)
  printf("%7lu) ## NK [%s:%d] [-] \n", millis(), __func__, __LINE__);
#endif
}

#endif /* SERIAL_INPUT */
