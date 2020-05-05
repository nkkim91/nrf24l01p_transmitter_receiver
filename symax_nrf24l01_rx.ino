#ifdef PROTOCOL_SYMA_X5C_RECEIVER_MODE

static u32 unRxPacketCounter;

static uint32_t NumValidAddressPacket;
#ifdef NK_RX_PKT_STAT
static uint32_t unChStat[MAX_RF_CHANNELS];
#endif
static uint8_t  ucRxTXAddrLockedIn = 0;

static uint32_t unChSetStat[MAX_RF_CHANNELS];

#if defined(CONFIG_SYMA_X5C_RECEIVER_NK) || defined(CONFIG_SYMA_X5C_RECEIVER_CAR2WD_NK)

static uint8_t SymaxX5CRxAirData[MAX_PACKET_SIZE];

#endif

static uint8_t receive_address_packet(uint32_t unTimeOut)
{
  uint8_t i, j, ucAbortBinding, ucPktPerChBalanced;
  volatile uint8_t ucStatus;
  unsigned long unBind2TimeOut, unLastTimeStamp;

#if defined(NKD_FUNC)
  printf("%7lu) ## NK [%s:%d] [+]\n", millis(), __func__, __LINE__);
#endif


  unBind2TimeOut = millis() + unTimeOut; /* 14 sec : 820 pkts, 28 sec : 1000 pkts */
  unLastTimeStamp = millis() + RX_RF_ADDR_HOPPING_PERIOD;

  // clear packet status bits and TX FIFO
  NRF24L01_WriteReg(NRF24L01_07_STATUS, (1 << NRF24L01_07_RX_DR)    // reset the flag(s) - Data Ready RX FIFO interrupt | Data Sent TX FIFO interrupt | Maximum number of TX retransmits interrupt
                    | (1 << NRF24L01_07_TX_DS)
                    | (1 << NRF24L01_07_MAX_RT));  /* 0x70 */
                    
  NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_MASK_RX_DR)   // switch to RX mode (CRC0 - 2 bytes)
                    | (1 << NRF24L01_00_MASK_MAX_RT) 
                    | (1 << NRF24L01_00_EN_CRC)
                    | (1 << NRF24L01_00_CRCO)
                    | (1 << NRF24L01_00_PWR_UP)
                    | (1 << NRF24L01_00_PRIM_RX) );  /* 0x5F */

  NRF24L01_WriteReg(NRF24L01_05_RF_CH, chans[ucCurrentChan]);  /* 75, 48, 64, 32 or 9 */
  NRF24L01_FlushRx();

  for( ucAbortBinding = 0, ucPktPerChBalanced = 0; !ucAbortBinding && (long)(millis() - unBind2TimeOut) < 0 ; ) {

#if defined(NKD_DEBUG_VERBOSE)
    printf("%7lu) ## NK [%s:%d] pkt %ld: chans[%d] : %d\n", millis(), __func__, __LINE__, unRxPacketCounter, ucCurrentChan, chans[ucCurrentChan]);
#endif

    /* Check if the RF channel needs to hop */
    if( (long)(millis() - unLastTimeStamp) >= 0 ) {
      /*******************************************************************************
       *
       * TX sends a packet in every 4 ms and changes RF CH in every 2 pkt.
       * TX transmit packet over 4 RF channels (75.48,64, 32 or 9)
       * 
       * Change the RF CH of RX in every 32 ms so that RX receives at least 2 pkts in every 32 ms
       * 
       * 8 ms hopping misses the packet more than 32 ms because the timer and the function doesn't run precisely in sync with TX.
       *
       *******************************************************************************/
      if( unRxPacketCounter++ % 2 ) {
        ucCurrentChan = (ucCurrentChan + 1) % num_rf_channels;
      }
      NRF24L01_WriteReg(NRF24L01_05_RF_CH, chans[ucCurrentChan]);  /* 75, 48, 64, 32(?) */
      NRF24L01_FlushRx();
      
      unLastTimeStamp = millis() + RX_RF_ADDR_HOPPING_PERIOD;  /* 32 (?) */
#ifdef SYMA_X5C_LED
      /*******************************************************************************
       * LED blinks fast until 4 sec.and slow down it by 4 after then.
       *******************************************************************************/
      if( unLedState++ < BIND_FASTLED_TIMEOUT / RX_RF_ADDR_HOPPING_PERIOD ) {
        if( unLedState % 2 ) {  /* Blink fast */
          digitalWrite(gnLED1Pin, 1);
        } else {
          digitalWrite(gnLED1Pin, 0);        
        }
      } else {            /* Blink slow */
        if( !(unLedState % 8) ) {
          digitalWrite(gnLED1Pin, 1);
        } else {
          digitalWrite(gnLED1Pin, 0);
        }        
      }
#endif      
    } else {
      // Nothing to do as of now
    }
    
    ucStatus = NRF24L01_ReadReg(NRF24L01_07_STATUS);
#if defined(NKD_DEBUG_VERBOSE)
    printf("%7lu) ## NK [%s:%d] ucStatus : 0x%02x\n", millis(), __func__, __LINE__, ucStatus);
#endif

    /* 1. RX_DR bit indicating new data arrival to the RX_FIFO should be set 
     *  and 
     * 2. RX_P_NO bit(RX data pipe number) should not valid number, not RX FIFO empty.
     */
    /* NK - need to check if fifo_status register is full as well */
    if ( ucStatus & (1 << NRF24L01_07_RX_DR) && (((ucStatus >> NRF24L01_07_RX_P_NO) & 0x07) >= NRF24L01_RX_P_0 && ((ucStatus >> NRF24L01_07_RX_P_NO) & 0x07) <= NRF24L01_RX_P_5)) {

#if defined(NKD_DEBUG_VERBOSE)
      printf("%7lu) ## NK [%s:%d] ucStatus : 0x%02x\n", millis(), __func__, __LINE__, ucStatus);
#endif
        
      NRF24L01_WriteReg(NRF24L01_07_STATUS, (1 << NRF24L01_07_RX_DR) );    // reset the flag(s)
  
      if ( ucStatus & (1 << NRF24L01_07_TX_DS) ) {
        NRF24L01_WriteReg(NRF24L01_07_STATUS, (1 << NRF24L01_07_TX_DS) );    // reset the flag(s)
      }

      memset(packet, 0, packet_size);
      NRF24L01_ReadPayload(packet, packet_size);  // packet size : 10

      /****************************************************************
       * Check if address packet is received correctly
       *
       *   packet[0] = rx_tx_addr[4];
       *   packet[1] = rx_tx_addr[3];
       *   packet[2] = rx_tx_addr[2];
       *   packet[3] = rx_tx_addr[1];
       *   packet[4] = rx_tx_addr[0];
       *   packet[5] = 0xaa;
       *   packet[6] = 0xaa;
       *   packet[7] = 0xaa;
       *   packet[8] = 0x00;
       *   packet[9] = checksum
       ****************************************************************/
      if( (packet[5] == 0xaa) && (packet[6] == 0xaa) && (packet[7] == 0xaa) && (packet[8] == 0x00) && (packet[9] == SYMAX_checksum(packet)) ) {

        if( !ucRxTXAddrLockedIn ) {
          for( i = 0; i < RX_ADDR_SIZE; i++ ) {
            ucLockedRxTxAddr[RX_ADDR_SIZE-1-i] = packet[i];
          }
          ucRxTXAddrLockedIn = 1;
          continue;
        }

        /**************************************************************
         *
         * Verify if addr. pkt are received from each RF CH to trust
         *
         **************************************************************/
        if( ucRxTXAddrLockedIn ) {
          for( i = 0; i < RX_ADDR_SIZE; i++ ) {
            if( ucLockedRxTxAddr[RX_ADDR_SIZE-1-i] != packet[i] ) {
              // Invalid address. Abort the binding
              ucAbortBinding = 1;
              break;
            } else {
            }
          }
          if( !ucAbortBinding ) {
#ifdef NK_RX_PKT_STAT
            unChStat[ucCurrentChan]++;
#endif
            NumValidAddressPacket++;
            
            for( j = 0, ucPktPerChBalanced = 1; j < num_rf_channels; j++ ) {
#ifdef NK_RX_PKT_STAT
              if( unChStat[j] < MIN_RX_ADDR_PKTS ) {
                ucPktPerChBalanced = 0;
                break;
              }
#else

#endif
            }
          }
        } // RxTxAddrLockedIn
      } else {
#if defined(NKD_DEBUG)
        printf("%7lu) ## NK [%s:%d] Received packet is NOT valid !!\n", millis(), __func__, __LINE__);
        printf("%7lu) ## NK [%s:%d] CH:%d(%d)] 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", millis(), __func__, __LINE__, ucCurrentChan, chans[ucCurrentChan], packet[0], packet[1], packet[2], packet[3], packet[4], packet[5], packet[6], packet[7], packet[8], packet[9]);
#endif
      }
    } // Packet available
//      delay(1);

    if( ucPktPerChBalanced ) {  /* RxTxAddrLockedIn && ucPktPerChBalanced */
      break;  
    } 
  }

#if defined(NKD_DEBUG)
  printf("%7lu) ## NK [%s:%d] RxTxAddr(ucLockedRxTxAddr) : ", millis(), __func__, __LINE__);
  
  for( i = 0; i < RX_ADDR_SIZE; i++ ){
    printf("0x%02x ", ucLockedRxTxAddr[i]);
  }
  printf("\n");
#endif

#ifdef NK_RX_PKT_STAT
#if defined(NKD_DEBUG)
  printf("%7lu) ## NK [%s:%d] CH Stat. : ", millis(), __func__, __LINE__);
  for( i = 0; i < num_rf_channels; i++ ) {
    printf("%3d:0x%02x ", chans[i], unChStat[i]);
  }
  printf("\n");
#endif
#endif

#if defined(NKD_DEBUG)
  printf("%7lu) ## NK [%s:%d] RF Channels are %s balanced !!\n", millis(), __func__, __LINE__, ucPktPerChBalanced ? "" : "NOT");
  printf("%7lu) ## NK [%s:%d] %d Valid address packet !! \n", millis(), __func__, __LINE__, NumValidAddressPacket);
#endif

#if defined(NKD_FUNC)
  printf("%7lu) ## NK [%s:%d] [-] \n", millis(), __func__, __LINE__);
#endif

  return ucPktPerChBalanced;
}

#ifdef NK_RX_PKT_STAT
static uint32_t  unStartTime = millis();
static uint32_t  unNumValidPacket = 0;
static uint32_t  unNumInvalidPacket = 0;
#endif

uint8_t  ucSymaxReceivedDataReady = 0;
static unsigned long unLastTimeStamp = 0;

static uint8_t receive_symax_x5c_data_packet(uint32_t unTimeOut)
{
  uint8_t i, j;
  volatile uint8_t ucStatus;
  static uint8_t ucOncePassed = 0;

#if defined(NKD_FUNC)
  printf("%7lu) ## NK [%s:%d] [+] \n", millis(), __func__, __LINE__);
#endif

//  unBind2TimeOut = millis() + unTimeOut; /* 14 sec : 820 pkts, 28 sec : 1000 pkts */

  noInterrupts();
  ucSymaxReceivedDataReady = 0; // Data is invalid, hence mark with invalid.
  interrupts();

  if( !ucOncePassed ) {

    // clear packet status bits and TX FIFO
    NRF24L01_WriteReg(NRF24L01_07_STATUS, (1 << NRF24L01_07_RX_DR)    //reset the flag(s)
                    | (1 << NRF24L01_07_TX_DS)
                    | (1 << NRF24L01_07_MAX_RT));  /* 0x70 */
                    
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_MASK_RX_DR)   // switch to RX mode
                    | (1 << NRF24L01_00_MASK_MAX_RT) 
                    | (1 << NRF24L01_00_EN_CRC)
                    | (1 << NRF24L01_00_CRCO)
                    | (1 << NRF24L01_00_PWR_UP)
                    | (1 << NRF24L01_00_PRIM_RX) );  /* 0x5F */


    NRF24L01_WriteReg(NRF24L01_05_RF_CH, chans[ucCurrentChan]);  /* 4 RF CH. selected from SYMAX_set_channels() */
    NRF24L01_FlushRx();

#ifdef NK_RX_PKT_STAT
    unStartTime = millis();
    unNumValidPacket = 0;
    unNumInvalidPacket = 0;
#endif

    unLastTimeStamp = millis() + RX_RF_DATA_HOPPING_PERIOD;

    ucOncePassed = 1;
  }

  for( ; 1 ; ) {

#if defined(NKD_DEBUG_VERBOSE)
    printf("%7lu) ## NK [%s:%d] pkt %ld: chans[%d] : %d \n", millis(), __func__, __LINE__, unRxPacketCounter, ucCurrentChan, chans[ucCurrentChan]);
#endif

    if( (long)(millis() - unLastTimeStamp) >= 0 ) {
      /*******************************************************************************
       *
       * TX sends a packet in every 4 ms and changes RF CH in every 2 pkt.
       * TX transmit packet over 4 RF channels selected from SYMAX_set_channels
       * 
       * Change the RF CH of RX in every 32 ms so that RX receives at least 2 pkts in every 32 ms
       * 
       * 8 ms hopping misses the packet more than 32 ms because the timer and the function doesn't run precisely in sync with TX.
       *
       *******************************************************************************/
      if( unRxPacketCounter++ % 2 ) {        
        ucCurrentChan = (ucCurrentChan + 1) % num_rf_channels;
      }
      NRF24L01_WriteReg(NRF24L01_05_RF_CH, chans[ucCurrentChan]);
//      NRF24L01_FlushRx(); // NK - testing
      
      unChSetStat[ucCurrentChan]++;
            
      unLastTimeStamp = millis() + RX_RF_DATA_HOPPING_PERIOD;  /* 32 (?) */

    } else {
      // Nothing to do as of now
    }
    
    ucStatus = NRF24L01_ReadReg(NRF24L01_07_STATUS);
#if defined(NKD_DEBUG_VERBOSE)
    printf("%7lu) ## NK [%s:%d] ucStatus : 0x%02x\n", millis(), __func__, __LINE__, ucStatus);
#endif

    /* 1. RX_DR bit indicating new data arrival to the RX_FIFO should be set 
     *  and 
     * 2. RX_P_NO bit(RX data pipe number) should not valid number, not RX FIFO empty.
     */
    if ( ucStatus & (1 << NRF24L01_07_RX_DR) && (((ucStatus >> NRF24L01_07_RX_P_NO) & 0x07) >= NRF24L01_RX_P_0 && ((ucStatus >> NRF24L01_07_RX_P_NO) & 0x07) <= NRF24L01_RX_P_5)) {

#if defined(NKD_DEBUG_VERBOSE)
      printf("%7lu) ## NK [%s:%d] ucStatus : 0x%02x\n", millis(), __func__, __LINE__, ucStatus);
#endif
        
      NRF24L01_WriteReg(NRF24L01_07_STATUS, (1 << NRF24L01_07_RX_DR) );    // reset the flag(s)
  
      if ( ucStatus & (1 << NRF24L01_07_TX_DS) ) {
        NRF24L01_WriteReg(NRF24L01_07_STATUS, (1 << NRF24L01_07_TX_DS) );    // reset the flag(s)
      }

      memset(SymaxX5CRxAirData, 0, packet_size);
      NRF24L01_ReadPayload(SymaxX5CRxAirData, packet_size);  // packet size : 10

      /****************************************************************
       * Check if data packet is received correctly
       *
       *   SymaxX5CRxAirData[0] = throttle
       *   SymaxX5CRxAirData[1] = elevator
       *   SymaxX5CRxAirData[2] = rudder
       *   SymaxX5CRxAirData[3] = aileron
       *   SymaxX5CRxAirData[4] = bit 7 - FLAG_VIDEO, bit 6 - FLAG_PICTURE, 
       *   SymaxX5CRxAirData[5] = (elevator >> 2 | 0xC0)  // 0x43 (?) -> 0xC3 
       *   SymaxX5CRxAirData[6] = (rudder >> 2) | (ucFlags & FLAG_FLIP  ? 0x40 : 0x00)
       *   SymaxX5CRxAirData[7] = (aileron >> 2)  | (ucFlags & FLAG_HEADLESS ? 0x80 : 0x00)
       *   SymaxX5CRxAirData[8] = 0x00;
       *   SymaxX5CRxAirData[9] = checksum
       ****************************************************************/
      if( (SymaxX5CRxAirData[8] == 0x00) && (SymaxX5CRxAirData[9] == SYMAX_checksum(SymaxX5CRxAirData)) ) {

#if defined(NKD_DEBUG_VERBOSE)
      printf("%7lu) ## NK [%s:%d] CH:%d] ", millis(), __func__, __LINE__, ucCurrentChan);
      for(int i=0; i < 6; i++) {
        printf("0x%02x ", SymaxX5CRxAirData[i]);
      }
      printf("\n");
#endif
        noInterrupts();
        ucSymaxReceivedDataReady = 1;
        interrupts();

#ifdef SYMA_X5C_LED
        /*******************************************************************************
         * 
         *******************************************************************************/
        if( unLedState++ % 2 ) {
          digitalWrite(gnLED1Pin, 1);
          digitalWrite(gnLED2Pin, 0);
        } else {
          digitalWrite(gnLED1Pin, 0);
          digitalWrite(gnLED2Pin, 0);          
        }
#endif   
#ifdef NK_RX_PKT_STAT
        unNumValidPacket++;
        unChStat[ucCurrentChan]++;
#endif

#ifdef MOTOR_PWM
        calculate_motor_pwm();

        analogWrite(gucMotorPwm_Pin1, ucMotorPwm[MOTOR_1_IDX]);  /* Pin 5 */
#if 0        
        analogWrite(gucMotorPwm_Pin2, adjust_pwm_pin5_6[ucMotorPwm[MOTOR_2_IDX]/10] + ucMotorPwm[MOTOR_2_IDX] % 10);
#else        
        analogWrite(gucMotorPwm_Pin2, ucMotorPwm[MOTOR_2_IDX]);
#endif        

#if 0        
        analogWrite(gucMotorPwm_Pin3, adjust_pwm_pin5_6[ucMotorPwm[MOTOR_3_IDX]/10] + ucMotorPwm[MOTOR_3_IDX] % 10);
#else
        analogWrite(gucMotorPwm_Pin3, ucMotorPwm[MOTOR_3_IDX]);
#endif        

        analogWrite(gucMotorPwm_Pin4, ucMotorPwm[MOTOR_4_IDX]);  /* Pin 6 */
#endif /* MOTOR_PWM */

      } else {

#if defined(NKD_DEBUG)
        printf("%7lu) ## NK [%s:%d] CH:%d] Received data packet is not valid (ucStatus:0x%02x) !! - ", millis(), __func__, __LINE__, ucCurrentChan, ucStatus);
        for(int i=0; i < 6; i++) {
          printf("0x%02x ", SymaxX5CRxAirData[i]);
        }
        printf("\n");
#endif

        noInterrupts();
        ucSymaxReceivedDataReady = 0; // Data is invalid, hence mark with invalid.
        interrupts();

#ifdef SYMA_X5C_LED
        /*******************************************************************************
         * 
         *******************************************************************************/
        if( unLedState++ % 2 ) {
          digitalWrite(gnLED1Pin, 0);
          digitalWrite(gnLED2Pin, 1);
        } else {
          digitalWrite(gnLED1Pin, 0);
          digitalWrite(gnLED2Pin, 0);
        }
#endif     
#ifdef NK_RX_PKT_STAT
        unNumInvalidPacket++;
#endif        
      }
      
    } // Packet available
    else { // Fifo empty
//      delay(4);
      if( ((ucStatus >> NRF24L01_07_RX_P_NO) & 0x07) == NRF24L01_RX_P_EMPTY ) {
#if defined(NKD_DEBUG_VERBOSE)
        printf("%7lu) ## NK [%s:%d] RX Fifo empty ucStatus : 0x%02x\n", millis(), __func__, __LINE__, ucStatus);
#endif
      }
#if 0
      noInterrupts();
      ucSymaxReceivedDataReady = 0; // Data is invalid, hence mark with invalid.
      interrupts();
#endif
      /* NK - Need to check further for the code flow */
      goto IDLE_RET;
    }
  }

IDLE_RET:
#ifdef NK_RX_PKT_STAT
    if( millis() - unStartTime > NK_RX_PKT_STAT_CYCLE ) {  /* 10 sec */

      printf("%7lu) ## NK [%s:%d] Valid Packet : %d, Invalid Packet : %d\n", millis(), __func__, __LINE__, unNumValidPacket, unNumInvalidPacket);

      printf("%7lu) ## NK [%s:%d] CH Stat. : ", millis(), __func__, __LINE__);
      for( i = 0; i < num_rf_channels; i++ ) {
//        printf("%d:0x%02x ", chans[i], unChStat[i]);
//        printf("%d:0x%02x ", chans[i], unChSetStat[i]);
        printf("%d:0x%02x ", chans[i], unChStat[i] );
      }
      printf("\n");

      // NK - When merge this one and above, printf() doens't work as expected. Not found the reason		
      printf("%7lu) ## NK [%s:%d] CH Setting Stat. : ", millis(), __func__, __LINE__);
      for( i = 0; i < num_rf_channels; i++ ) {
        printf("%d:0x%02x ", chans[i], unChSetStat[i]);
      }
      printf("\n");

      unStartTime = millis();
      unNumValidPacket = 0;
      unNumInvalidPacket = 0;
      for(i = 0; i < num_rf_channels; i++) {
        unChStat[i] = 0;
        unChSetStat[i] = 0;
      }
    }
#endif

#if defined(NKD_FUNC)
  printf("%7lu) ## NK [%s:%d] [-] \n", millis(), __func__, __LINE__);
#endif

  return 0;
}

#ifdef CONFIG_HMC5883L_GY273_RECEIVER_NK

static uint8_t receive_hmc5883l_data_packet(uint32_t unTimeOut) /* argument not used */
{
  uint8_t i, j;
  volatile uint8_t ucStatus;
  unsigned long unLastTimeStamp;

#if defined(NKD_FUNC)
  printf("%7lu) ## NK [%s:%d] [+] \n", millis(), __func__, __LINE__);
#endif

//  unBind2TimeOut = millis() + unTimeOut; /* 14 sec : 820 pkts, 28 sec : 1000 pkts */
  unLastTimeStamp = millis() + RX_RF_DATA_HOPPING_PERIOD;

  // clear packet status bits and TX FIFO
  NRF24L01_WriteReg(NRF24L01_07_STATUS, (1 << NRF24L01_07_RX_DR)    //reset the flag(s)
                    | (1 << NRF24L01_07_TX_DS)
                    | (1 << NRF24L01_07_MAX_RT));  /* 0x70 */
                    
  NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_MASK_RX_DR)   // switch to RX mode
                    | (1 << NRF24L01_00_MASK_MAX_RT) 
                    | (1 << NRF24L01_00_EN_CRC)
                    | (1 << NRF24L01_00_CRCO)
                    | (1 << NRF24L01_00_PWR_UP)
                    | (1 << NRF24L01_00_PRIM_RX) );  /* 0x5F */


  NRF24L01_WriteReg(NRF24L01_05_RF_CH, chans[ucCurrentChan]);  /* 4 RF CH. selected from SYMAX_set_channels() */
  NRF24L01_FlushRx();

#ifdef NK_RX_PKT_STAT
  unStartTime = millis();
  unNumValidPacket = 0;
  unNumInvalidPacket = 0;
#endif

  for( ; 1 ; ) {

#if defined(NKD_DEBUG_VERBOSE)
    printf("%7lu) ## NK [%s:%d] pkt %ld: chans[%d] : %d\n", millis(), __func__, __LINE__, unRxPacketCounter, ucCurrentChan, chans[ucCurrentChan]);
#endif

    if( (long)(millis() - unLastTimeStamp) >= 0 ) {
      /*******************************************************************************
       *
       * TX sends a packet in every 4 ms and changes RF CH in every 2 pkt.
       * TX transmit packet over 4 RF channels selected from SYMAX_set_channels
       * 
       * Change the RF CH of RX in every 32 ms so that RX receives at least 2 pkts in every 32 ms
       * 
       * 8 ms hopping misses the packet more than 32 ms because the timer and the function doesn't run precisely in sync with TX.
       *
       *******************************************************************************/
      if( unRxPacketCounter++ % 2 ) {
        ucCurrentChan = (ucCurrentChan + 1) % num_rf_channels;  
      }
      NRF24L01_WriteReg(NRF24L01_05_RF_CH, chans[ucCurrentChan]);
      NRF24L01_FlushRx();
      
      unLastTimeStamp = millis() + RX_RF_DATA_HOPPING_PERIOD;  /* 32 (?) */

    } else {
      // Nothing to do as of now
    }
    
    ucStatus = NRF24L01_ReadReg(NRF24L01_07_STATUS);
#if defined(NKD_DEBUG_VERBOSE)
    printf("%7lu) ## NK [%s:%d] ucStatus : 0x%02x\n", millis(), __func__, __LINE__, ucStatus);
#endif

    /* 1. RX_DR bit indicating new data arrival to the RX_FIFO should be set 
     *  and 
     * 2. RX_P_NO bit(RX data pipe number) should not valid number, not RX FIFO empty.
     */
    if ( ucStatus & (1 << NRF24L01_07_RX_DR) && (((ucStatus >> NRF24L01_07_RX_P_NO) & 0x07) >= NRF24L01_RX_P_0 && ((ucStatus >> NRF24L01_07_RX_P_NO) & 0x07) <= NRF24L01_RX_P_5)) {

#if defined(NKD_DEBUG_VERBOSE)
      printf("%7lu) ## NK [%s:%d] ucStatus : 0x%02x\n", millis(), __func__, __LINE__, ucStatus);
#endif
        
      NRF24L01_WriteReg(NRF24L01_07_STATUS, (1 << NRF24L01_07_RX_DR) );    //reset the flag(s)
  
      if ( ucStatus & (1 << NRF24L01_07_TX_DS) ) {
        NRF24L01_WriteReg(NRF24L01_07_STATUS, (1 << NRF24L01_07_TX_DS) );    //reset the flag(s)
      }

      memset(gstEcompassRxAirPacketData.ucByte, 0, packet_size);
      NRF24L01_ReadPayload(gstEcompassRxAirPacketData.ucByte, packet_size);  // packet size : 10

      /****************************************************************
       * Check if data packet is received correctly
       *
       ****************************************************************/
      if( (gstEcompassRxAirPacketData.stEcompassAirData.unMagic == AIR_DATA_PKT_MAGIC) && (gstEcompassRxAirPacketData.ucByte[9] == SYMAX_checksum(gstEcompassRxAirPacketData.ucByte)) ) {
#if defined(NKD_DEBUG_VERBOSE)
        printf("%7lu) ## NK [%s:%d] CH:%d Heading:%d\n", millis(), __func__, __LINE__, ucCurrentChan, gstEcompassRxAirPacketData.stEcompassAirData.gfCompassHeading);
#endif

#ifdef LCD1602_NK
        float headingDegrees = gstEcompassRxAirPacketData.stEcompassAirData.gfCompassHeading;
        lcd.setCursor(0, 0);
        lcd.print("                ");
        lcd.setCursor(0, 1);
        lcd.print("                ");

        lcd.setCursor(0, 1);
        lcd.print((uint16_t)headingDegrees,DEC);
          
        lcd.setCursor(4, 1);
        if( headingDegrees > 337.5 || headingDegrees < 22.5 ) {
          lcd.print("N");
        } else if( headingDegrees >= 22.5 && headingDegrees < 67.5 ) {
          lcd.print("NE");
        } else if( headingDegrees >= 67.5  && headingDegrees < 112.5 ) {
          lcd.print("E");
        } else if( headingDegrees >= 112.5  && headingDegrees < 157.5 ) {
          lcd.print("SE");
        } else if( headingDegrees >= 157.5  && headingDegrees < 202.5 ) {
          lcd.print("S");
        } else if( headingDegrees >= 202.5  && headingDegrees < 247.5 ) {
          lcd.print("SW");
        } else if( headingDegrees >= 247.5  && headingDegrees < 292.5 ) {
          lcd.print("W");
        } else if( headingDegrees >= 292.5  && headingDegrees < 337.5 ) {
          lcd.print("NW");
        }          
#endif          
#ifdef SYMA_X5C_LED
        /*******************************************************************************
         * 
         *******************************************************************************/
        if( unLedState++ % 2 ) {
          digitalWrite(gnLED1Pin, 1);
          digitalWrite(gnLED2Pin, 0);
        } else {
          digitalWrite(gnLED1Pin, 0);
          digitalWrite(gnLED2Pin, 0);          
        }
#endif   
#ifdef NK_RX_PKT_STAT
        unNumValidPacket++;
#endif        

      } else {
#if defined(NKD_DEBUG)
        printf("%7lu) ## NK [%s:%d] CH:%d - Received data packet is not valid (ucStatus:0x%02x) !! - 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", millis(), __func__, __LINE__, ucCurrentChan, ucStatus, gstEcompassRxAirPacketData.ucByte[0], gstEcompassRxAirPacketData.ucByte[1], gstEcompassRxAirPacketData.ucByte[2], gstEcompassRxAirPacketData.ucByte[3], gstEcompassRxAirPacketData.ucByte[4], gstEcompassRxAirPacketData.ucByte[5], gstEcompassRxAirPacketData.ucByte[6], gstEcompassRxAirPacketData.ucByte[7], gstEcompassRxAirPacketData.ucByte[8], gstEcompassRxAirPacketData.ucByte[9]);
#endif
#ifdef SYMA_X5C_LED
        /*******************************************************************************
         * 
         *******************************************************************************/
        if( unLedState++ % 2 ) {
          digitalWrite(gnLED1Pin, 0);
          digitalWrite(gnLED2Pin, 1);
        } else {
          digitalWrite(gnLED1Pin, 0);
          digitalWrite(gnLED2Pin, 0);
        }
#endif     
#ifdef NK_RX_PKT_STAT
        unNumInvalidPacket++;
#endif        
      }

    } // Packet available
    else {
//      delay(4);
      if( ((ucStatus >> NRF24L01_07_RX_P_NO) & 0x07) == NRF24L01_RX_P_EMPTY ) {
#if defined(NKD_DEBUG_VERBOSE)
        printf("%7lu) ## NK [%s:%d] RX Fifo empty ucStatus : 0x%02x\n", millis(), __func__, __LINE__, ucStatus);
#endif
      }
    }

#ifdef NK_RX_PKT_STAT
    if( millis() - unStartTime > 10000 ) {  /* 10 sec */
#if defined(NKD_DEBUG)
      printf("%7lu) ## NK [%s:%d] Valid Packet : %lu, Invalid Packet : %lud\n", millis(), __func__, __LINE__, unNumValidPacket, unNumInvalidPacket); 
#endif
      unStartTime = millis();
      unNumValidPacket = 0;
      unNumInvalidPacket = 0;
    }
#endif    
  }

#if defined(NKD_FUNC)
  printf("%7lu) ## NK [%s:%d] [-] \n", millis(), __func__, __LINE__);
#endif

  return 0;
}
#endif /* CONFIG_HMC5883L_GY273_RECEIVER_NK */


static void symax_receiver_loop()
{
  uint8_t  ucRet = -1;
  uint8_t i;

#if defined(NKD_FUNC)
  printf("%7lu) ## NK [%s:%d] [+] \n", millis(), __func__, __LINE__);
#endif

#if defined(NKD_DEBUG_VERBOSE)
  printf("%7lu) ## NK [%s:%d] phase : %d\n", millis(), __func__, __LINE__, phase);
#endif

#ifdef CONFIG_HMC5883L_GY273_RECEIVER_NK
#ifdef LCD1602_NK

  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print("                ");

  lcd.setCursor(0, 0); lcd.print("E-Compass");  
  lcd.setCursor(0, 1); lcd.print("Waiting data....");

#endif /* LCD1602_NK */
#endif /* CONFIG_HMC5883L_GY273_RECEIVER_NK */
  
  while ( 1 ) {

    switch(phase) {
      case SYMAX_INIT1:
        symax_init1();
        phase = SYMAX_BIND2;
        break;
  
      case SYMAX_BIND2:
        counter = BIND_COUNT;  /* NK, 345 * 4 ms = 1.38 sec */
        phase = SYMAX_BIND3;
#if defined(NKD_DEBUG_VERBOSE)
    printf("%7lu) ## NK [%s:%d] Now binding ... \n", millis(), __func__, __LINE__);
#endif
        break;
  
      case SYMAX_BIND3:
        ucRet = receive_address_packet(BIND_TIMEOUT);
#ifdef NRF24L01_RECEIVER_CHANNEL_BALANCE_CHECK_NK
        if( !ucRet ) {  /* Packet is not balanced through the RF CH or timeout */
          printf("%7lu) ## NK [%s:%d] Recvd data pkt NOT balanced !!\n", millis(), __func__, __LINE__);
          return;
          while(1);  /* Stop receiver */
        } else {
          printf("%7lu) ## NK [%s:%d] Now moving on to next !!\n", millis(), __func__, __LINE__);
        }
#else
        printf("%7lu) ## NK [%s:%d] Ignore whether CH is balanced or NOT !!\n", millis(), __func__, __LINE__);
#endif

        symax_init2();  /* Set new RF CH */
        phase = SYMAX_DATA;
//        PROTOCOL_SetBindState(0);
#ifdef MELODY_NK
//        MUSIC_Play(MELODY_SUCCESS);  /* NK, seems tone library conflicts with the timer */
#endif

#ifdef NK_RX_PKT_STAT // Reset the stat. array before receiving data packet
        for(i = 0; i < num_rf_channels; i++) {
          unChStat[i] = 0;
          unChSetStat[i] = 0;
        }
#endif        
        break;
  
      case SYMAX_DATA:
#ifdef CONFIG_HMC5883L_GY273_RECEIVER_NK
        ucRet = receive_hmc5883l_data_packet(1);
#else
        ucRet = receive_symax_x5c_data_packet(1);
#endif
        break;    
        
      default : 
        break;
    } // switch

    if( ucRet == 0 ) {
      break;
    }
  } // while

#if defined(NKD_FUNC)
  printf("%7lu) ## NK [%s:%d] [-]\n", millis(), __func__, __LINE__);
#endif
}

#endif /* PROTOCOL_SYMA_X5C_RECEIVER_MODE */
