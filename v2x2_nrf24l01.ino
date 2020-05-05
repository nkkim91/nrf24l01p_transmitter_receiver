#ifdef V2X2_MODEL

/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
// compatible with WLToys V2x2, JXD JD38x, JD39x, JJRC H6C, Yizhan Tarantula X6 ...
// Last sync with hexfet new_protocols/v202_nrf24l01.c dated 2015-03-15



//#include "iface_nrf24l01.h"
#include "v2x2_nrf24l01.h"
#include "TX_Def.h"


uint8_t packet_sent;

uint8_t hopping_frequency_no=0;
uint8_t rx_tx_addr[5];
uint8_t hopping_frequency[50];
uint8_t  *hopping_frequency_ptr;
uint16_t Channel_data[NUM_CHN];
uint8_t Channel_AUX;
uint8_t sub_protocol = JXD506;
uint8_t phase = 0;
uint16_t bind_counter;
uint8_t protocol_flags=0;

/* Telemetry */
uint8_t RxData[16];
uint8_t *packet = RxData;
uint8_t flags=0;


// Channel value 100% is converted to 8bit values 0<->255
uint8_t convert_channel_8b(uint8_t num)
{
    uint16_t val=Channel_data[num];
    val=((val<<2)+val)>>5;
    if(val<=32) return 0;
    if(val>=288) return 255;
    return val-32;
}

// Channel value is converted sign + magnitude 8bit values
uint8_t convert_channel_s8b(uint8_t num)
{
    uint8_t ch;
    ch = convert_channel_8b(num);
    return (ch < 128 ? 127-ch : ch);
}

// This is frequency hopping table for V202 protocol
// The table is the first 4 rows of 32 frequency hopping
// patterns, all other rows are derived from the first 4.
// For some reason the protocol avoids channels, dividing
// by 16 and replaces them by subtracting 3 from the channel
// number in this case.
// The pattern is defined by 5 least significant bits of
// sum of 3 bytes comprising TX id
const uint8_t PROGMEM freq_hopping[][16] = {
	{ 0x27, 0x1B, 0x39, 0x28, 0x24, 0x22, 0x2E, 0x36,
		0x19, 0x21, 0x29, 0x14, 0x1E, 0x12, 0x2D, 0x18 }, //  00
	{ 0x2E, 0x33, 0x25, 0x38, 0x19, 0x12, 0x18, 0x16,
		0x2A, 0x1C, 0x1F, 0x37, 0x2F, 0x23, 0x34, 0x10 }, //  01
	{ 0x11, 0x1A, 0x35, 0x24, 0x28, 0x18, 0x25, 0x2A,
		0x32, 0x2C, 0x14, 0x27, 0x36, 0x34, 0x1C, 0x17 }, //  02
	{ 0x22, 0x27, 0x17, 0x39, 0x34, 0x28, 0x2B, 0x1D,
		0x18, 0x2A, 0x21, 0x38, 0x10, 0x26, 0x20, 0x1F }  //  03
};

static void __attribute__((unused)) v202_init()
{
	NRF24L01_Initialize();

	// 2-bytes CRC, radio off
	NRF24L01_WriteReg(NRF24L01_00_CONFIG, _BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO)); 
	NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknoledgement
	NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x3F);  // Enable all data pipes
	NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);   // 5-byte RX/TX address
	NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0xFF); // 4ms retransmit t/o, 15 tries
	NRF24L01_WriteReg(NRF24L01_05_RF_CH, 0x08);      // Channel 8
	NRF24L01_SetBitrate(NRF24L01_BR_1M);                          // 1Mbps
#if 0	
	NRF24L01_SetPower();
#else
  NRF24L01_SetPower(TXPOWER_150mW);
#endif 
	
	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
	//    NRF24L01_WriteReg(NRF24L01_08_OBSERVE_TX, 0x00); // no write bits in this field
	//    NRF24L01_WriteReg(NRF24L01_00_CD, 0x00);         // same
	NRF24L01_WriteReg(NRF24L01_0C_RX_ADDR_P2, 0xC3); // LSB byte of pipe 2 receive address
	NRF24L01_WriteReg(NRF24L01_0D_RX_ADDR_P3, 0xC4);
	NRF24L01_WriteReg(NRF24L01_0E_RX_ADDR_P4, 0xC5);
	NRF24L01_WriteReg(NRF24L01_0F_RX_ADDR_P5, 0xC6);
	NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, V2X2_PAYLOADSIZE);   // bytes of data payload for pipe 1
	NRF24L01_WriteReg(NRF24L01_12_RX_PW_P1, V2X2_PAYLOADSIZE);
	NRF24L01_WriteReg(NRF24L01_13_RX_PW_P2, V2X2_PAYLOADSIZE);
	NRF24L01_WriteReg(NRF24L01_14_RX_PW_P3, V2X2_PAYLOADSIZE);
	NRF24L01_WriteReg(NRF24L01_15_RX_PW_P4, V2X2_PAYLOADSIZE);
	NRF24L01_WriteReg(NRF24L01_16_RX_PW_P5, V2X2_PAYLOADSIZE);
	NRF24L01_WriteReg(NRF24L01_17_FIFO_STATUS, 0x00); // Just in case, no real bits to write here
	NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, (uint8_t *)"\x66\x88\x68\x68\x68", 5);
	NRF24L01_WriteRegisterMulti(NRF24L01_0B_RX_ADDR_P1, (uint8_t *)"\x88\x66\x86\x86\x86", 5);
	NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, (uint8_t *)"\x66\x88\x68\x68\x68", 5);
}

/* RF24 library doesn't support it */
static uint8_t NRF24L01_packet_ack()
{
    switch (NRF24L01_ReadReg(NRF24L01_07_STATUS) & (_BV(NRF24L01_07_TX_DS) | _BV(NRF24L01_07_MAX_RT)))
    {   
        case _BV(NRF24L01_07_TX_DS):
            return PKT_ACKED;
        case _BV(NRF24L01_07_MAX_RT):
            return PKT_TIMEOUT;
    }   
    return PKT_PENDING;
}

/* ---------------------------------------------------------------------------------------------------------------- */
#define BABS(X) (((X) < 0) ? -(u8)(X) : (X))

/* Going into reverse is enabled if the throttle value increases from 0.
 * but, if the raw throttle value from the joystick is not centered to 0, minus value such as 0x82 could be returned. 
 * It's negative for goint into reverse. but, the A959 doesn't go into revserse.
 * So, adjust/calibrate the value from the joystick.
 */
#define JOYSTICK_RAW_MAX  (1023)
#define JOYSTICK_RAW_MIN  (0)
#define JOYSTICK_CALIBRATION_TIMES (3)

#define CHANNEL_DATA_MAX  (255) /* Max. in 1 byte length */
#define CHANNEL_DATA_MIN  (0)
#define CHANNEL_DATA_MID  ((CHANNEL_DATA_MAX - CHANNEL_DATA_MIN + 1)/2 - 1) // 127

#if 0
// Lower the max. throttle for safety.
#define CHANNEL_DATA_LIMIT (32)  /* Throttle */
#else
#define CHANNEL_DATA_LIMIT (127)  /* Normal */
#endif

#if !defined(SERIAL_INPUT)
static uint16_t usJoystickRawMid; // joystick_raw_mid;
static int16_t usJoystickCalibrationDelta; // joystick_calibration_delta;

/***************************************************
 *
 * Calculate the calibration-delta with the raw joystick input range(0~1023)
 *
 ***************************************************/
static void Calibrate_Throttle()
{
  uint8_t i;
  uint16_t usJoystickRawMidSum, usDefaultMid;

  for( i = 0, usJoystickRawMidSum = 0; i < JOYSTICK_CALIBRATION_TIMES; i++) {
    usJoystickRawMidSum += analogRead(gnM2R_yPin);  /* pitch/elevator/mode2_right_y */
    delay(20);
  }
  usJoystickRawMid = usJoystickRawMidSum / JOYSTICK_CALIBRATION_TIMES;

  usDefaultMid = (JOYSTICK_RAW_MAX - JOYSTICK_RAW_MIN) / 2;
  
  if( (usJoystickRawMid - usDefaultMid) > 0 ) {
    usJoystickCalibrationDelta = usJoystickRawMid - usDefaultMid;
  } else {
    usJoystickCalibrationDelta = -(usJoystickRawMid - usDefaultMid);
  }

#if defined(NKD_DEBUG)
  printf("%7lu) ## NK [%s:%d] Joystick THR. Raw Mid : %d, Calibation Delta : %d\n", millis(), __func__, __LINE__, usJoystickRawMid, usJoystickCalibrationDelta);
#endif
}

static void V2X2_read_controls()
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

  gnPosRAdjX = gnPosRX;

#if defined(CONFIG_SYMA_X5C_TRANSMITTER_ELEPHANT_NK) || defined(CONFIG_SYMA_X5C_TRANSMITTER_PLUS_NK)
  gnPosRAdjX = 255 - gnPosRAdjX - 128;
#else
  gnPosRAdjX = 2048 - gnPosRAdjX * 2; // Scale up the value (0 ~ 1023 ==> 0 ~ 2047)
#endif

  Channel_data[AILERON] = gnPosRAdjX;

#if defined(NKD_SPOT)
  printf("%7lu) ## NK [%s:%d] gnPosRX:%04d/gnPosRAdjX:%04d\n", millis(), __func__, __LINE__, gnPosRX, gnPosRAdjX);
#endif

#ifdef THROTTLE_CALIBRATION
  gnPosRAdjY = gnPosRY - usJoystickCalibrationDelta;
  
  if( gnPosRAdjY < JOYSTICK_RAW_MIN )
    gnPosRAdjY = JOYSTICK_RAW_MIN;
  if( gnPosRAdjY > JOYSTICK_RAW_MAX )
      gnPosRAdjY = JOYSTICK_RAW_MAX;
#endif  /* THROTTLE_CALIBRATION */

#if defined(CONFIG_SYMA_X5C_TRANSMITTER_ELEPHANT_NK) || defined(CONFIG_SYMA_X5C_TRANSMITTER_PLUS_NK)
  gnPosRAdjY = 255 - gnPosRAdjY - 128;  /* Forward (high), Backward (Low) */
#else
  gnPosRAdjY = gnPosRAdjY * 2; // Scale up the value (0 ~ 1023 ==> 0 ~ 2047)
#endif

  Channel_data[ELEVATOR] = gnPosRAdjY; /* 0 ~ 1023 => 0 ~ 255 */

#if defined(NKD_SPOT)
  printf("%7lu) ## NK [%s:%d] gnPosRY:%04d/gnPosRAdjY:%04d\n", millis(), __func__, __LINE__, gnPosRY, gnPosRAdjY);
#endif

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

  gnPosLAdjX = gnPosLX;
  gnPosLAdjY = gnPosLY;

#if defined(CONFIG_SYMA_X5C_TRANSMITTER_ELEPHANT_NK) || defined(CONFIG_SYMA_X5C_TRANSMITTER_PLUS_NK)
  gnPosLAdjX = 255 - gnPosLAdjX - 128;
#else
  gnPosLAdjX = 2048 - gnPosLAdjX * 2;
#endif
  Channel_data[RUDDER] = gnPosLAdjX;

#if defined(NKD_SPOT)
  printf("%7lu) ## NK [%s:%d] gnPosLX:%04d/gnPosLAdjX:%04d\n", millis(), __func__, __LINE__, gnPosLX, gnPosLAdjX);
#endif

#if defined(CONFIG_SYMA_X5C_TRANSMITTER_ELEPHANT_NK) || defined(CONFIG_SYMA_X5C_TRANSMITTER_PLUS_NK)
  gnPosLAdjY = 255 - gnPosLAdjY;
#else
  gnPosLAdjY = 2048 - gnPosLAdjY * 2;
#endif
  Channel_data[THROTTLE] = gnPosLAdjY;              /* throttle/mode2_left_y */ /* AETR (0123) Channel_data[1] = Left Y,  */

#if defined(NKD_SPOT)
  printf("%7lu) ## NK [%s:%d] gnPosLY:%04d/gnPosLAdjY:%04d)\n", millis(), __func__, __LINE__, gnPosLY, gnPosLAdjY);
#endif

#if 0

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
#endif  // JOY_BUTTON

#if defined(NKD_SPOT)
    printf("%7lu) ## NK [%s:%d] [%1d/%1d/%1d/%1d] (%02x:%04x) (%02x:%02x) ", millis(), __func__, __LINE__, AILERON, ELEVATOR, THROTTLE, RUDDER, Channel_data[RUDDER], Channel_data[THROTTLE], Channel_data[AILERON], Channel_data[ELEVATOR]);
    printf("gnPosLX/gnPosLAdjX = (%d:%d), gnPosLY/gnPosLAdjY = (%d:%d), gnPosRX/gnPosRAdjX = (%d:%d), gnPosRY/gnPosRAdjY = (%d:%d)\n", gnPosLX, gnPosLAdjX, gnPosLY, gnPosLAdjY, gnPosRX, gnPosRAdjX, gnPosRY, gnPosRAdjY);
#endif

  /* NK - helpful for tunning */
#if defined(NKD_DELTA)
    if( gnPosLAdjX_Prev != gnPosLAdjX || gnPosLAdjY_Prev != gnPosLAdjY || gnPosRAdjX_Prev != gnPosRAdjX || gnPosRAdjY_Prev != gnPosRAdjY ) {
      printf("gnPosLX/gnPosLAdjX = (%d:%d), gnPosLY/gnPosLAdjY = (%d:%d), gnPosRX/gnPosRAdjX = (%d:%d), gnPosRY/gnPosRAdjY = (%d:%d)\n", gnPosLX, gnPosLAdjX, gnPosLY, gnPosLAdjY, gnPosRX, gnPosRAdjX, gnPosRY, gnPosRAdjY);

      gnPosLX_Prev = gnPosLX; gnPosLAdjX_Prev = gnPosLAdjX;
      gnPosLY_Prev = gnPosLY; gnPosLAdjY_Prev = gnPosLAdjY;
      gnPosRX_Prev = gnPosRX; gnPosRAdjX_Prev = gnPosRAdjX;
      gnPosRY_Prev = gnPosRY; gnPosRAdjY_Prev = gnPosRAdjY;
    }
#endif
}
#endif /* !SEIAL_INPUT */

static void __attribute__((unused)) V202_init2()
{
  NRF24L01_FlushTx();
  packet_sent = 0;
  hopping_frequency_no = 0;

  // Turn radio power on
  NRF24L01_SetTxRxMode(TX_EN);
  //Done by TX_EN??? => NRF24L01_WriteReg(NRF24L01_00_CONFIG, _BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));
}

static void __attribute__((unused)) V2X2_set_tx_id(void)
{
  uint8_t sum;
  sum = rx_tx_addr[1] + rx_tx_addr[2] + rx_tx_addr[3];
  // Higher 3 bits define increment to corresponding row
  uint8_t increment = (sum & 0x1e) >> 2;
  // Base row is defined by lowest 2 bits
  sum &=0x03;
  for (uint8_t i = 0; i < 16; ++i) {
    uint8_t val = pgm_read_byte_near(&freq_hopping[sum][i]) + increment;
    // Strange avoidance of channels divisible by 16
    hopping_frequency[i] = (val & 0x0f) ? val : val - 3;
  }
}

static void __attribute__((unused)) V2X2_add_pkt_checksum()
{
  uint8_t sum = 0;
  for (uint8_t i = 0; i < 15;  ++i)
    sum += packet[i];
  packet[15] = sum;
}

static void __attribute__((unused)) V2X2_send_packet(uint8_t bind)
{
  uint8_t flags2=0;

#ifdef SERIAL_INPUT
  V2X2_read_control_serial();
#else
  V2X2_read_controls();
#endif
  
  if (bind)
  {
    flags     = V2X2_FLAG_BIND;
    packet[0] = 0;
    packet[1] = 0;
    packet[2] = 0;
    packet[3] = 0;
    packet[4] = 0;
    packet[5] = 0;
    packet[6] = 0;
  }
  else
  {
#if defined(NKD_DEBUG_VERBOSE)
    printf("%7lu) ## NK [%s:%d] Elevator : %u(0x%02x)\n", millis(), __func__, __LINE__, Channel_data[ELEVATOR], Channel_data[ELEVATOR]);
    printf("%7lu) ## NK [%s:%d] Aileron  : %u(0x%02x)\n", millis(), __func__, __LINE__, Channel_data[AILERON], Channel_data[AILERON]);
#endif

    packet[0] = convert_channel_8b(THROTTLE);
    packet[1] = convert_channel_s8b(RUDDER);
    packet[2] = convert_channel_s8b(ELEVATOR); /* -127 ~ 127 */
    packet[3] = convert_channel_s8b(AILERON); /* -127 ~ 127 */
    // Trims, middle is 0x40
    packet[4] = 0x40; // yaw trim
    packet[5] = 0x40; // pitch trim
    packet[6] = 0x40; // roll trim

		//Flags
		flags=0;
		// Channel 5
		if (CH5_SW)	flags = V2X2_FLAG_FLIP;
		// Channel 6
		if (CH6_SW)	flags |= V2X2_FLAG_LIGHT;
		// Channel 7
		if (CH7_SW)	flags |= V2X2_FLAG_CAMERA;
		// Channel 8
		if (CH8_SW)	flags |= V2X2_FLAG_VIDEO;

		//Flags2
		// Channel 9
		if (CH9_SW)
			flags2 = V2X2_FLAG_HEADLESS;
		if(sub_protocol==JXD506)
		{
			// Channel 11
			if (CH11_SW)
				flags2 |= V2X2_FLAG_EMERGENCY;
		}
		else
		{
			// Channel 10
			if (CH10_SW)
				flags2 |= V2X2_FLAG_MAG_CAL_X;
			// Channel 11
			if (CH11_SW)
				flags2 |= V2X2_FLAG_MAG_CAL_Y;
		}
	}
	// TX id
	packet[7] = rx_tx_addr[1];
	packet[8] = rx_tx_addr[2];
	packet[9] = rx_tx_addr[3];
	// flags
	packet[10] = flags2;
	packet[11] = 0x00;
	packet[12] = 0x00;
	packet[13] = 0x00;
	if(sub_protocol==JXD506)
	{
		// Channel 10
		if (CH10_SW)
			packet[11] = V2X2_FLAG_START_STOP;
		// Channel 12
		if(CH12_SW)
			packet[11] |= V2X2_FLAG_CAMERA_UP;
		else if(Channel_data[CH12] < CHANNEL_MIN_COMMAND)
			packet[11] |= V2X2_FLAG_CAMERA_DN;
		packet[12] = 0x40;
		packet[13] = 0x40;
	}
	packet[14] = flags;
	V2X2_add_pkt_checksum();

	packet_sent = 0;
	uint8_t rf_ch = hopping_frequency[hopping_frequency_no >> 1];
	hopping_frequency_no = (hopping_frequency_no + 1) & 0x1F;
	NRF24L01_WriteReg(NRF24L01_05_RF_CH, rf_ch);
	NRF24L01_FlushTx();
	NRF24L01_WritePayload(packet, V2X2_PAYLOADSIZE);
	packet_sent = 1;

	if (! hopping_frequency_no)
#if 0
		NRF24L01_SetPower();
#else
    NRF24L01_SetPower(TXPOWER_150mW);
#endif
}

uint16_t ReadV2x2()	// remote_callback
{

#if defined(NKD_DEBUG)
    printf("%7lu) ## NK [%s:%d] phase : %d\n", millis(), __func__, __LINE__, phase);
#endif

  switch (phase) {
    case V202_INIT2:
      V202_init2();
      phase = V202_BIND2;
#if 1
      Timer1.initialize(150);
      Timer1.attachInterrupt(ReadV2x2);

#if defined(NKD_DEBUG)
      printf("%7lu) ## NK [%s:%d] V202_INIT2\n", millis(), __func__, __LINE__);
#endif
      return 0;
#else
      return 150;
#endif
      break;

    case V202_INIT2_NO_BIND:
      V202_init2();
      phase = V202_DATA;
#if 1
      Timer1.initialize(150);
      Timer1.attachInterrupt(ReadV2x2);

#if defined(NKD_DEBUG)
      printf("%7lu) ## NK [%s:%d] V202_INIT2_NO_BIND \n", millis(), __func__, __LINE__);
#endif
      return 0;
#else
      return 150;
#endif
      break;

    case V202_BIND2:
      if (packet_sent && NRF24L01_packet_ack() != PKT_ACKED) {
#if defined(NKD_DEBUG)
        printf("%7lu) ## NK [%s:%d] Packet NACK \n", millis(), __func__, __LINE__);
#endif
#if 1
        Timer1.initialize(V2X2_PACKET_CHKTIME);
        Timer1.attachInterrupt(ReadV2x2);
        return 0;
#else
        return V2X2_PACKET_CHKTIME;
#endif
      }

#if defined(NKD_DEBUG)
      printf("%7lu) ## NK [%s:%d] V202_BIND2 - Pkt. Sent : %u, Bind Count : %u \n", millis(), __func__, __LINE__, packet_sent, bind_counter);
#endif
			
      V2X2_send_packet(1);
      if (--bind_counter == 0) {
        phase = V202_DATA;
        BIND_DONE;
      }
      break;

    case V202_DATA:
      if (packet_sent && NRF24L01_packet_ack() != PKT_ACKED) {
#if 1
        Timer1.initialize(V2X2_PACKET_CHKTIME);
        Timer1.attachInterrupt(ReadV2x2);

#if defined(NKD_DEBUG)
        printf("%7lu) ## NK [%s:%d] V202_DATA\n", millis(), __func__, __LINE__);
#endif
        return 0;
#else
        return V2X2_PACKET_CHKTIME;
#endif
      }
      V2X2_send_packet(0);
      break;
  } // switch
	// Packet every 4ms
#if 1
  Timer1.initialize(V2X2_PACKET_PERIOD);
  Timer1.attachInterrupt(ReadV2x2);

#if defined(NKD_DEBUG)
  printf("%7lu) ## NK [%s:%d] \n", millis(), __func__, __LINE__);
#endif
 
  return 0;
#else
  return V2X2_PACKET_PERIOD;
#endif
}

uint16_t initV2x2()	// next_callback
{	
#if defined(NKD_DEBUG)
    printf("%7lu) [+] ## NK [%s:%d]\n", millis(), __func__, __LINE__);
#endif

  v202_init();
  //
  if ( 1 || IS_BIND_IN_PROGRESS)
  {
    bind_counter = V2X2_BIND_COUNT;
    phase = V202_INIT2;
  }
  else {
    phase = V202_INIT2_NO_BIND;
  }

  V2X2_set_tx_id();

#if 1
  Timer1.initialize(50000);
  Timer1.attachInterrupt(ReadV2x2);

#if defined(NKD_DEBUG)
  printf("%7lu) [-] ## NK [%s:%d]\n", millis(), __func__, __LINE__);
#endif
  return 0;
#else
  return 50000;
#endif
}

#endif  /* V2X2_MODEL */

