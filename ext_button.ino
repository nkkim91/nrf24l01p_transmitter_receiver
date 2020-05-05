#ifdef EXT_BUTTON

union TxPacketData gstTxPacketData, gstRxPacketData, gstTempRxPacketData;
eRX_DATA_RET_t geRxDataRet;

/***************************************************
 *
 * External Button Functions
 *
 ***************************************************/

PROGMEM const uint32_t crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};

unsigned long crc_update(unsigned long crc, byte data)
{
    byte tbl_idx;
    tbl_idx = crc ^ (data >> (0 * 4));
    crc = pgm_read_dword_near(crc_table + (tbl_idx & 0x0f)) ^ (crc >> 4);
    tbl_idx = crc ^ (data >> (1 * 4));
    crc = pgm_read_dword_near(crc_table + (tbl_idx & 0x0f)) ^ (crc >> 4);
    return crc;
}

unsigned long crc_string(char *s, u8 length)
{
  u8 i;
  unsigned long crc = ~0L;
  for( i = 0; i < length; i++) {
    crc = crc_update(crc, *s++);
  }  
  crc = ~crc;
  return crc;
}

void TX_DataPkt(void) 
{
  u8 i;
  
  for( i = 0; i < TX_PACKET_DATA_SIZE; i++ ) {
    Serial.write(gstTxPacketData.ucByte[i]);
  }
}

/***************************************************
 * RX_DataPkt 
 * 
 * Max time : RX_PKT_TIMEOUT
 * Min time : 
 *
 * SERIAL (Sub Board - Trim switches) -> gstTempRxPacketData
 * 
 ***************************************************/
eRX_DATA_RET_t RX_DataPkt(void) 
{
  int nRet;
  eRX_DATA_RET_t eRet = RX_DATA_VALID;  /* RX_DATA_VALID : 0 */
  u8 i, j;
  unsigned long ulTimeOut;
  unsigned long ulCRC;
  
  ulTimeOut = millis() + RX_PKT_TIMEOUT;  /* */
  
  i = 0;
  while ( i < TX_PACKET_DATA_SIZE ) {

    if( (long)(millis() - ulTimeOut) < 0 ) {
      if( j = Serial.available() ) {

#if defined(NKD_SPOT)
	printf("%7lu) ## NK [%s:%d] (i=%d) Available:%d\n", i, j);
#endif

        nRet = Serial.read();
        if( nRet != -1 ) {
          if( i == 0 && nRet != (DATA_PKT_MAGIC & 0xFF) ) {
            continue;
          }
          else {
            gstTempRxPacketData.ucByte[i] = nRet & 0xFF;
            ulTimeOut = millis() + RX_PKT_TIMEOUT;  /* */
            i++;
          }
        } else {
          continue; // No data available
        }
      }
      else {
        continue;  // wait for the incoming data
      }
    } else {
      eRet = RX_DATA_TIMEOUT;  /* RX_DATA_TIMEOUT : -1 */
      break;
    }
  }

#if defined(NKD_DEBUG)
  printf("%7lu) ## NK [%s:%d] ", millis(), __func__, __LINE__);
  for( i = 0; i < TX_PACKET_DATA_SIZE; i++ ) {
    printf("0x%02x ", gstTempRxPacketData.ucByte[i]);
  }
  printf("\n");
#endif

  if( eRet >= RX_DATA_VALID ) {

    ulCRC = crc_string((char *)&gstTempRxPacketData.stTxData, TX_PACKET_DATA_SIZE - 4);
    if (ulCRC != gstTempRxPacketData.stTxData.u.ulCRC ) {
      eRet = RX_DATA_CRC_ERROR;
#if defined(NKD_SPOT)
      printf("%7lu) ## NK [%s:%d] Packet Data Invalid !!\n", millis(), __func__, __LINE__);
#endif
    }
    else {
#if defined(NKD_DEBUG_VERBOSE)
      printf("%7lu) ## NK [%s:%d] Rudder   : %d\n", millis(), __func__, __LINE__, gstTempRxPacketData.stTxData.stTrimData.cRudder);
      printf("%7lu) ## NK [%s:%d] Elevator : %d\n", millis(), __func__, __LINE__, gstTempRxPacketData.stTxData.stTrimData.cElevator);
      printf("%7lu) ## NK [%s:%d] Aileron  : %d\n", millis(), __func__, __LINE__, gstTempRxPacketData.stTxData.stTrimData.cAileron);
#endif

      // Check min & max
      if( gstTempRxPacketData.stTxData.stTrimData.cRudder < MIN_RUDDER_TRIM || 
        gstTempRxPacketData.stTxData.stTrimData.cRudder > MAX_RUDDER_TRIM ) {
        eRet = RX_DATA_MINMAX_ERROR;
      } else if( gstTempRxPacketData.stTxData.stTrimData.cElevator < MIN_ELEVATOR_TRIM || 
        gstTempRxPacketData.stTxData.stTrimData.cElevator > MAX_ELEVATOR_TRIM ) {
        eRet = RX_DATA_MINMAX_ERROR;
      } else if( gstTempRxPacketData.stTxData.stTrimData.cAileron < MIN_AILERON_TRIM || 
        gstTempRxPacketData.stTxData.stTrimData.cAileron > MAX_AILERON_TRIM ) {
        eRet = RX_DATA_MINMAX_ERROR;
      }
#if defined(NKD_SPOT)
      printf("%0lu) ## NK [%s:%d] Packet Data Valid !! LED(Available:%d)\n", millis(), __func__, __LINE__, Serial.available());
#endif
    }
  }

  return eRet;
}
#endif /* EXT_BUTTON */

