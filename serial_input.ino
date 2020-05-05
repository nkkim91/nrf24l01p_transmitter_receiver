#ifdef SERIAL_INPUT

#include "nrf24l01p_transmitter.h"
#include "serial_input.h"

union ChannelPacketData_b gstTxChannelPacketData, gstTempTxChannelPacketData;
union ChannelPacketData_b gstRxChannelPacketData, gstTempRxChannelPacketData;

eERR_CH_PKT_DATA_t geErrChannelPktData;

/***************************************************
 *
 * External Button Functions
 *
 ***************************************************/
static PROGMEM const uint32_t crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};

static unsigned long crc_update(unsigned long crc, byte data)
{
    byte tbl_idx;
    uint8_t idx;
    
//    printf("[1] crc : 0x%08lx, data : %d\n", crc, data);
    tbl_idx = (crc ^ (data >> (0 * 4)));
//    printf("[2] tbl_idx : %d, tbl_idx & 0x0f : %d, crc >> 4 : 0x%08lx\n", tbl_idx, tbl_idx & 0x0f, crc >> 4);
    crc = crc_table[(tbl_idx & 0x0f)] ^ (crc >> 4);
//    printf("[3] crc : 0x%08lx\n", crc);
    tbl_idx = crc ^ (data >> (1 * 4));
//    printf("[4] tbl_idx : %d\n", tbl_idx);
    crc = crc_table[(tbl_idx & 0x0f)] ^ (crc >> 4);
//    printf("[5] crc : 0x%08lx\n", crc);
    return crc;
}

static unsigned long crc_string(char *s, u8 length)
{
  u8 i;
  unsigned long crc = ~0L;
  for( i = 0; i < length; i++) {
    crc = crc_update(crc, *s++);
  }  
  crc = ~crc;
  return crc;
}

static void ntohl(uint32_t* value) {
 uint32_t tmp_a = (*value & 0xff000000) >> 24;
 uint32_t tmp_b = (*value & 0x00ff0000) >> 8;
 uint32_t tmp_c = (*value & 0x0000ff00) << 8 ;
 uint32_t tmp_d = (*value & 0x000000ff) << 24;
 *value = tmp_d | tmp_c |tmp_b | tmp_a;
}

/* 
 *  SERIAL INPUT -> gstTempRxChannelPacketData
 *  
 */
int RX_ChannelPktData(void) 
{
  int nRet;
  int eRet = ERR_CH_PKT_DATA_VALID;  /* RX_DATA_VALID : 0 */
  u8 i, j;
  unsigned long ulTimeOut;
  unsigned long ulCRC;
  
  ulTimeOut = millis() + RX_CH_PKT_DATA_TIMEOUT;  /* */
  
  memset(&gstTempRxChannelPacketData, 0x0, sizeof(union ChannelPacketData_b));
  
  i = 0;
  while ( i < CHANNEL_PACKET_DATA_SIZE ) {

    if( (long)(millis() - ulTimeOut) < 0 ) {

      if( j = Serial.available() ) {
        
#if defined(NKD_SPOT)
        printf("%7lu) ## NK [%s:%d] (i=%d)Available:%d\n", millis(), __func__, __LINE__, i, j);
#endif

        nRet = Serial.read();
//        printf("%c\n", nRet);
        if( nRet != -1 ) {
          if( i == 0 && nRet != ((CH_DATA_PKT_MAGIC & 0xFF) >> 0) ) {
            continue;
          } else if ( i == 1 && nRet != ((CH_DATA_PKT_MAGIC & 0xFF00) >> 8) ) {
#if defined(NKD_DEBUG)
              printf("%7lu) ## NK [%s:%d] MAGIC maismatch [i = %d] %x, nRet = %x\n", millis(), __func__, __LINE__, i, gstTempRxChannelPacketData.ucByte[0], nRet);
#endif

            eRet = ERR_CH_PKT_DATA_MAGIC_ERROR;
            break;
          } else if ( i == 2 && nRet != ((CH_DATA_PKT_MAGIC & 0xFF0000) >> 16) ) {
#if defined(NKD_DEBUG)
              printf("%7lu) ## NK [%s:%d] MAGIC maismatch [i = %d] %x, %x nRet = %x\n", millis(), __func__, __LINE__, i, gstTempRxChannelPacketData.ucByte[0], gstTempRxChannelPacketData.ucByte[1], nRet);
#endif
            eRet = ERR_CH_PKT_DATA_MAGIC_ERROR;
            break;
          } else if ( i == 3 && nRet != ((CH_DATA_PKT_MAGIC & 0xFF000000) >> 24) ) {
#if defined(NKD_DEBUG)

              printf("%7lu) ## NK [%s:%d] MAGIC maismatch [i = %d] %x, %x, %x nRet = %x\n", millis(), __func__, __LINE__, i, gstTempRxChannelPacketData.ucByte[0], gstTempRxChannelPacketData.ucByte[1], gstTempRxChannelPacketData.ucByte[2], nRet);
#endif
            eRet = ERR_CH_PKT_DATA_MAGIC_ERROR;
            break;
          } else {
            gstTempRxChannelPacketData.ucByte[i] = nRet & 0xFF;
            ulTimeOut = millis() + RX_CH_PKT_DATA_TIMEOUT;  /* */
            i++;
          }
        } else {
          printf("%07lu) ## NK [%s:%d] Read error !!\n", millis(), __func__, __LINE__);
          continue; // No data available
        }
      }
      else {
        continue;  // wait for the incoming data
      }
    } else {
      eRet = ERR_CH_PKT_DATA_TIMEOUT;  /* RX_DATA_TIMEOUT : -1 */
      break;
    }
  }

#if defined(NKD_DEBUG)
    if( eRet != ERR_CH_PKT_DATA_TIMEOUT ) {
      printf("%7lu) ## NK [%s:%d] ", millis(), __func__, __LINE__);
      for( i = 0; i < CHANNEL_PACKET_DATA_SIZE; i++ ) {
        printf("0x%02x ", gstTempRxChannelPacketData.ucByte[i]);
      }
      printf("\n");
    }
#endif

  if( eRet >= ERR_CH_PKT_DATA_VALID ) {
    ulCRC = crc_string((char *)&gstTempRxChannelPacketData.stChannelPacketData, CHANNEL_PACKET_DATA_SIZE - sizeof(union CRCData)); /* */
    if (ulCRC != gstTempRxChannelPacketData.stChannelPacketData.uCRCData.ulCRC ) {
      eRet = ERR_CH_PKT_DATA_CRC_ERROR;
#if defined(NKD_SPOT)
        printf("%7lu) ## NK [%s:%d] CRC calculated : 0x%08lx, CRC embedded : 0x%08lx\n", millis(), __func__, __LINE__, ulCRC, gstTempRxChannelPacketData.stChannelPacketData.uCRCData.ulCRC);
        printf("%7lu) ## NK [%s:%d] Packet Data invalid !!\n", millis(), __func__, __LINE__);
#endif
    }
    else {
#if defined(NKD_SPOT)
      printf("%7lu) ## NK [%s:%d] Aileron  : %d\n", millis(), __func__, __LINE__, gstTempRxChannelPacketData.stChannelPacketData.stChannelData.ucAileron);
      printf("%7lu) ## NK [%s:%d] Elevator : %d\n", millis(), __func__, __LINE__, gstTempRxChannelPacketData.stChannelPacketData.stChannelData.ucElevator);
      printf("%7lu) ## NK [%s:%d] Throttle : %d\n", millis(), __func__, __LINE__, gstTempRxChannelPacketData.stChannelPacketData.stChannelData.ucThrottle);
      printf("%7lu) ## NK [%s:%d] Rudder   : %d\n", millis(), __func__, __LINE__, gstTempRxChannelPacketData.stChannelPacketData.stChannelData.ucRudder);
      printf("%7lu) ## NK [%s:%d] Packet Data Valid !! \n", millis(), __func__, __LINE__);
#endif
    }
  }

  return eRet;
}

#endif /* SERIAL_INPUT */
