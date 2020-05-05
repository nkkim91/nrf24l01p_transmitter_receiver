#ifdef NRF24L01_DRIVER

/***************************************************
 *
 * nRF24L01 Driver Functions
 *
 ***************************************************/
static void CS_LO() {
  // Minimum ideal SPI bus speed is 2x data rate
  // If we assume 2Mbs data rate and 16Mhz clock, a
  // divider of 4 is the minimum we want.
  // CLK:BUS 8Mhz:2Mhz, 16Mhz:4Mhz, or 20Mhz:5Mhz
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  digitalWrite(gnNRF24L01_CSNPin,LOW);
}

static void CS_HI() {
  // Minimum ideal SPI bus speed is 2x data rate
  // If we assume 2Mbs data rate and 16Mhz clock, a
  // divider of 4 is the minimum we want.
  // CLK:BUS 8Mhz:2Mhz, 16Mhz:4Mhz, or 20Mhz:5Mhz

  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  digitalWrite(gnNRF24L01_CSNPin,HIGH);
}

void NRF24L01_Initialize()
{
    rf_setup = 0x0F;
}    

u8 NRF24L01_WriteReg(u8 reg, u8 data)
{
  CS_LO();
  uint8_t res = SPI.transfer( W_REGISTER | ( REGISTER_MASK & reg ) );
  SPI.transfer(data);
  CS_HI();
  return res;
}

u8 NRF24L01_WriteRegisterMulti(u8 reg, const u8 data[], u8 length)
{
    CS_LO();
    uint8_t res = SPI.transfer(W_REGISTER | ( REGISTER_MASK & reg));
    for (u8 i = 0; i < length; i++)
    {
        SPI.transfer(data[i]);
    }
    CS_HI();
    return res;
}

u8 NRF24L01_WritePayload(u8 *data, u8 length)
{
    CS_LO();
    uint8_t res = SPI.transfer(W_TX_PAYLOAD);
    for (u8 i = 0; i < length; i++)
    {
        SPI.transfer(data[i]);
    }
    CS_HI();
    return res;
}

u8 NRF24L01_ReadReg(u8 reg)
{
    CS_LO();
    SPI.transfer(R_REGISTER | (REGISTER_MASK & reg));
    uint8_t data = SPI.transfer(0xFF);
    CS_HI();
    return data;
}

u8 NRF24L01_ReadRegisterMulti(u8 reg, u8 data[], u8 length)
{
    CS_LO();
    uint8_t res = SPI.transfer(R_REGISTER | (REGISTER_MASK & reg));
    for(u8 i = 0; i < length; i++)
    {
        data[i] = SPI.transfer(0xFF);
    }
    CS_HI();
    return res;
}

u8 NRF24L01_ReadPayload(u8 *data, u8 length)
{
    CS_LO();
    uint8_t res = SPI.transfer(R_RX_PAYLOAD);
    for(u8 i = 0; i < length; i++)
    {
        data[i] = SPI.transfer(0xFF);
    }
    CS_HI();
    return res;
}

static u8 Strobe(u8 state)
{
    CS_LO();
    uint8_t res = SPI.transfer(state);
    CS_HI();
    return res;
}

u8 NRF24L01_FlushTx()
{
    return Strobe(FLUSH_TX);
}

u8 NRF24L01_FlushRx()
{
    return Strobe(FLUSH_RX);
}

u8 NRF24L01_Activate(u8 code)  /* NK */
{
    CS_LO();
    uint8_t res = SPI.transfer(ACTIVATE);  /* 0x50 */
    SPI.transfer(code);
    CS_HI();
    return res;
}

u8 NRF24L01_SetBitrate(uint8_t bitrate)
{
    // Note that bitrate 250kbps (and bit RF_DR_LOW) is valid only
    // for nRF24L01+. There is no way to programmatically tell it from
    // older version, nRF24L01, but the older is practically phased out
    // by Nordic, so we assume that we deal with with modern version.

    // Bit 0 goes to RF_DR_HIGH, bit 1 - to RF_DR_LOW
    rf_setup = (rf_setup & 0xD7) | ((bitrate & 0x02) << 4) | ((bitrate & 0x01) << 3);
    return NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, rf_setup);
}

// Power setting is 0..3 for nRF24L01
// Claimed power amp for nRF24L01 from eBay is 20dBm. 
//      Raw            w 20dBm PA
// 0 : -18dBm  (16uW)   2dBm (1.6mW)
// 1 : -12dBm  (60uW)   8dBm   (6mW)
// 2 :  -6dBm (250uW)  14dBm  (25mW)
// 3 :   0dBm   (1mW)  20dBm (100mW)
// So it maps to Deviation as follows
/*
TXPOWER_100uW  = -10dBm
TXPOWER_300uW  = -5dBm
TXPOWER_1mW    = 0dBm
TXPOWER_3mW    = 5dBm
TXPOWER_10mW   = 10dBm
TXPOWER_30mW   = 15dBm
TXPOWER_100mW  = 20dBm
TXPOWER_150mW  = 22dBm
*/

u8 NRF24L01_SetPower(u8 power)
{
    u8 nrf_power = 0;
    switch(power) {
        case TXPOWER_100uW: nrf_power = 0; break;
        case TXPOWER_300uW: nrf_power = 0; break;
        case TXPOWER_1mW:   nrf_power = 0; break;
        case TXPOWER_3mW:   nrf_power = 1; break;
        case TXPOWER_10mW:  nrf_power = 1; break;
        case TXPOWER_30mW:  nrf_power = 2; break;
        case TXPOWER_100mW: nrf_power = 3; break;
        case TXPOWER_150mW: nrf_power = 3; break;
        default:            nrf_power = 0; break;
    };
    // Power is in range 0..3 for nRF24L01
    rf_setup = (rf_setup & 0xF9) | ((nrf_power & 0x03) << 1);  /* NK : rf_setup = 0x0F */
    return NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, rf_setup);
}

static void CE_lo()
{
    digitalWrite(gnNRF24L01_CEPin, LOW);
}

static void CE_hi()
{
    digitalWrite(gnNRF24L01_CEPin, HIGH);
}

void NRF24L01_SetTxRxMode(uint8_t mode)
{
    if(mode == TX_EN) {
        CE_lo();
        NRF24L01_WriteReg(NRF24L01_07_STATUS, (1 << NRF24L01_07_RX_DR)    //reset the flag(s)
                                            | (1 << NRF24L01_07_TX_DS)
                                            | (1 << NRF24L01_07_MAX_RT));
        NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_EN_CRC)   // switch to TX mode
                                            | (1 << NRF24L01_00_CRCO)
                                            | (1 << NRF24L01_00_PWR_UP));
        usleep(130);
        CE_hi();
    } else if (mode == RX_EN) {
        CE_lo();
        NRF24L01_WriteReg(NRF24L01_07_STATUS, (1 << NRF24L01_07_RX_DR)    //reset the flag(s)
                                            | (1 << NRF24L01_07_TX_DS)
                                            | (1 << NRF24L01_07_MAX_RT));
        NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_EN_CRC)   // switch to RX mode
                                            | (1 << NRF24L01_00_CRCO)
                                            | (1 << NRF24L01_00_PWR_UP)
                                            | (1 << NRF24L01_00_PRIM_RX));
        usleep(130);
        CE_hi();
    } else {
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, ~(1 << NRF24L01_00_PWR_UP)); //PowerDown
        CE_lo();
    }
}

int NRF24L01_Reset()
{
    NRF24L01_FlushTx();
    NRF24L01_FlushRx();
    u8 status1 = Strobe(NOP);
    u8 status2 = NRF24L01_ReadReg(0x07);
    NRF24L01_SetTxRxMode(TXRX_OFF);
    return (status1 == status2 && (status1 & 0x0f) == 0x0e);
}

#ifdef NRF24L01_EXPERIMENT_FEATURE

void print_byte_register(const char *name, uint8_t reg, uint8_t qty)
{
  char extra_tab = strlen(name) < 8 ? '\t' : ' ';

  printf("%7lu) ## NK [%s:%d] \t%s\t%c", name, extra_tab);

  while ( qty--) {
    printf("0x%02x ", NRF24L01_ReadReg(reg++));
  }
  printf("\n");
}

void print_address_register(const char *name, uint8_t reg, uint8_t qty)
{
  uint8_t buffer[5];
  uint8_t i, j;
  char extra_tab = strlen(name) < 8 ? '\t' : ' ';

  printf("%7lu) ## NK [%s:%d] \t%s\t%c", name, extra_tab);

  while ( qty--) {

    NRF24L01_ReadRegisterMulti(reg++, buffer, 5);

    for (i = 0; i < 5; i++ ) {
      printf("0x%02x ", buffer[i]);
    }
    printf("  ");
  }
  printf("\n");
}

void NRF24L01_RegSummary(void)
{
  uint8_t ucDataRate, ucCRC, ucPALevel;

#if defined(NKD_INFO)
  printf("%7lu) ## NK [%s:%d] NRF24L01 Register summary \n", millis(), __func__, __LINE__);

  print_byte_register("STATUS", NRF24L01_07_STATUS, 1);
  print_address_register("RX_ADDR_P0-1", NRF24L01_0A_RX_ADDR_P0, 2);
  print_byte_register("RX_ADDR_P2-5", NRF24L01_0C_RX_ADDR_P2, 4);
  print_address_register("TX_ADDR", NRF24L01_10_TX_ADDR, 1);

  print_byte_register("RX_PW_P0-5", NRF24L01_11_RX_PW_P0, 6);
  print_byte_register("EN_AA", NRF24L01_01_EN_AA, 1);
  print_byte_register("EN_RXADDR", NRF24L01_02_EN_RXADDR, 1);
  print_byte_register("SETUP_AW", NRF24L01_03_SETUP_AW, 1);
  print_byte_register("SETUP_ETR", NRF24L01_04_SETUP_RETR, 1);  
  print_byte_register("RF_CH", NRF24L01_05_RF_CH, 1);
  print_byte_register("RF_SETUP", NRF24L01_06_RF_SETUP, 1);
  print_byte_register("CONFIG", NRF24L01_00_CONFIG, 1);
  print_byte_register("DYNPD/FEATURE", NRF24L01_1C_DYNPD, 2);
  print_byte_register("OBSERVE_TX", NRF24L01_08_OBSERVE_TX, 1);
  print_byte_register("FIFO_STATAUS", NRF24L01_17_FIFO_STATUS, 1);
#endif

  ucDataRate = NRF24L01_ReadReg(NRF24L01_06_RF_SETUP) & ((1 << NRF24L01_06_RF_DR_LOW) | (1 << NRF24L01_06_RF_DR_HIGH));

  printf("%7lu) ## NK [%s:%d] \tData Rate\t = ", millis(), __func__, __LINE__);
  switch (ucDataRate) {
    case (1 << NRF24L01_06_RF_DR_LOW) :
      printf("250KBPS\n");
      break;
    case (1 << NRF24L01_06_RF_DR_HIGH) :
      printf("2MBPS\n");
      break;
    case ((1 << NRF24L01_06_RF_DR_LOW) | (1 << NRF24L01_06_RF_DR_HIGH)) :
      printf("1MBPS\n");
      break;
  }

  ucCRC = NRF24L01_ReadReg(NRF24L01_00_CONFIG) & ((1 << NRF24L01_00_EN_CRC) | (1 << NRF24L01_00_CRCO));

  printf("%7lu) ## NK [%s:%d] ", millis(), __func__, __LINE__);

  if ( ucCRC & (1 << NRF24L01_00_EN_CRC) ) {
    if ( ucCRC & (1 << NRF24L01_00_CRCO) ) {
      printf("\tCRC16 \n");
    } else {
      printf("\tCRC8 \n");
    }
  } else {
    printf("\tCRC disabled !!\n");
  }

  ucPALevel = NRF24L01_ReadReg(NRF24L01_06_RF_SETUP) & ((1 << NRF24L01_06_PA_PWR_HIGH) | (1 << NRF24L01_06_PA_PWR_LOW));

  printf("%7lu) ## NK [%s:%d] ", millis(), __func__, __LINE__);
  switch (ucPALevel) {
    case ((1 << NRF24L01_06_PA_PWR_HIGH) | (1 << NRF24L01_06_PA_PWR_LOW)) :
      printf("\tPA_HIGH\n");
      break;
    case (1 << NRF24L01_06_PA_PWR_HIGH) :
      printf("\tPA_MED\n");
      break;
    case (1 << NRF24L01_06_PA_PWR_LOW) :
      printf("\tPA_LOW\n");
      break;
    default :
      printf("\tPA_MIN\n");
  }
}
#endif /* EXPERIMENT_FEATURE */
#endif /* NRF24L01_DRIVER */
