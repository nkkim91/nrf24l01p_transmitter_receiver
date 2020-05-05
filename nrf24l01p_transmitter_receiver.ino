/* 
 * 2015/05/22 - A1 line was disconnected from the PCB. And abnormal value was returned.
 * 2015/10/27 - When RAINBOW_LED is enabled, transmit is delayed due to the delay function call in the rainbow function.
 * 2015/11/15 - Regardless of calculation, 1000 ~ 1200 pkt are received/handled in 10 sec. => receive the pkt in every 9 ~ 10 ms
 *
 */

#include <SPI.h>
#include "printf.h"
#include "common.h"
#include "pitches.h"

#include "config.h"
#include "TX_Def.h"


#ifdef SYMAX_MODEL
#include "symax_nrf24l01.h"
#endif

#ifdef NK_DEBUG_TIMER_MONITOR
#define NK_DEBUG_TIMER_MONITOR_PERIOD	(2000)
unsigned long time_data_pkt_at;
unsigned long time_data_pkt;
#endif /* NK_DEBUG_TIMER_MONITOR */

#ifdef RAINBOW_LED
#include <Adafruit_NeoPixel.h>
#include "neopixel_ws2812.h"

extern Adafruit_NeoPixel pixels;

#endif /* RAINBOW_LED */

#ifdef EXT_BUTTON
#include "ext_button.h"

#endif /* EXT_BUTTON */

#ifdef LCD1602_NK
#include <LiquidCrystal.h>
#include "lcd1602.h"
#endif /* LCD1602_NK */

#ifdef MELODY_NK
#include "melody.h"
#endif /* MELODY_NK */

#ifdef SERVO_WHEEL_NK
#include "servo_wheel.h"
#endif /* SERVO_WHEEL_NK */

#ifdef CONFIG_HMC5883L_GY273_TRANSMITTER_NK
#include <Wire.h>
#include <HMC5883L.h>
#endif /* CONFIG_HMC5883L_GY273_TRANSMITTER_NK */


/***************************************************
 *
 * SYMA X5C LED Data structure / Macro
 *
 ***************************************************/
#ifdef SYMA_X5C_LED
uint32_t  unLedState;
#endif

/***************************************************
 *
 * HMC5883 magnentometer / compass Data structure / Macro
 *
 ***************************************************/

#ifdef CONFIG_HMC5883L_GY273_TRANSMITTER_NK

HMC5883L compass;

#endif /* CONFIG_HMC5883L_GY273_TRANSMITTER_NK */

#if defined(CONFIG_HMC5883L_GY273_TRANSMITTER_NK) || defined(CONFIG_HMC5883L_GY273_RECEIVER_NK)

#define ECOMPASS_AIR_DATA_PKT_MAGIC  (0xFADECAFE)

struct eCompassAirData {  // HMC5883L
  uint32_t unMagic;
  float    gfCompassHeading;
  uint8_t  ucOpt1;
  uint8_t  ucChecksum;  
};

#define ECOMPASS_AIR_PACKET_DATA_SIZE  sizeof(struct eCompassAirData)

union   eCompassAirPacketData {
  struct eCompassAirData stEcompassAirData;
  uint8_t ucByte[ECOMPASS_AIR_PACKET_DATA_SIZE];
};

union eCompassAirPacketData gstEcompassTxAirPacketData, gstEcompassRxAirPacketData;

#endif	/* CONFIG_HMC5883L_GY273_TRANSMITTER_NK || defined(CONFIG_HMC5883L_GY273_RECEIVER_NK) */

/***************************************************
 *
 * Control Data structure / Macro       
 *
 ***************************************************/
/***************************************************
 * Required for receiver
 ***************************************************/
#if defined(CONFIG_SYMA_X5C_RECEIVER_NK) || defined(CONFIG_SYMA_X5C_RECEIVER_CAR2WD_NK) || defined(CONFIG_HMC5883L_GY273_RECEIVER_NK)
#define FIRST_PACKET_TIMEOUT  (2000)  /* initially it was 500 ms */
#endif

#if defined(SYMA_X5C_LED)
const int gnLED1Pin = LED1_PIN;      // the number of the LED pin
const int gnLED2Pin = LED2_PIN;
#elif defined(NK_DEBUG_LED)
const int gnDebugLEDPin = NK_DEBUG_LED_PIN;
#elif defined(RAINBOW_LED)
const int gnRainbowLEDPin = RAINBOW_LED_PIN;
#endif


#ifdef GIMBAL_JOYSTICK_INPUT  // Transmitter has joystick input(gimbal)

#define ANALOG_INPUT_MAX  (1023)
#define ANALOG_INPUT_MIN  (0)

const int gnM2L_xPin = GIMBAL_JOYSTICK_LEFT_X_PIN;
const int gnM2L_yPin = GIMBAL_JOYSTICK_LEFT_Y_PIN;
const int gnM2R_xPin = GIMBAL_JOYSTICK_RIGHT_X_PIN;
const int gnM2R_yPin = GIMBAL_JOYSTICK_RIGHT_Y_PIN;

#endif /* GIMBAL_JOYSTICK_INPUT */

#ifdef JOY_BUTTON
const int gnM2L_ButtonPin = GIMBAL_JOYSTICK_BUTTON_LEFT_PIN;  // 7
const int gnM2R_ButtonPin = GIMBAL_JOYSTICK_BUTTON_RIGHT_PIN; // 8 
u8 key_state;
#endif /* JOY_BUTTON */

#ifdef SERVO_WHEEL_NK
uint32_t  unLastServoUpdateTime = millis();
uint32_t  unServoUpdateCount = 0;
#endif

#ifdef MOTOR_PWM
/***************************************************
 * Arduino PWM
 * 
 * The PWM outputs generated on pins 5 and 6 will have higher-than-expected duty cycles. 
 * This is because of interactions with the millis() and delay() functions, which share the same internal timer used to generate those PWM outputs. 
 * This will be noticed mostly on low duty-cycle settings (e.g 0 - 10) and may result in a value of 0 not fully turning off the output on pins 5 and 6. 
 ***************************************************/
#define NUM_MOTORS  (4)
#define NUM_MOTOR_INPUT  (3)  /* Rudder, Elevator, Aileron */

const uint8_t  gucMotorPwm_Pin1 = MOTOR_PIN1;  /* 5, 1st motor */
const uint8_t  gucMotorPwm_Pin2 = MOTOR_PIN2;  /* 3, 2nd motor */
const uint8_t  gucMotorPwm_Pin3 = MOTOR_PIN3;  /* 9, 3rd motor */
const uint8_t  gucMotorPwm_Pin4 = MOTOR_PIN4;  /* 6, 4th motor */
#endif /* MOTOR_PWM */

#ifdef NRF24L01_DRIVER
const uint8_t gnNRF24L01_CEPin = NRF24L01_CE_PIN;
const uint8_t gnNRF24L01_CSNPin = NRF24L01_CSN_PIN;
#endif /* NRF24L01_DRIVER */

#ifdef MELODY_NK
const int gnMelodyPin = MELODY_PIN;  // 4
#endif /* MELODY_NK */

#ifdef LCD1602_NK
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(LCD1602_DATA_PIN1, LCD1602_DATA_PIN2, LCD1602_DATA_PIN3, LCD1602_DATA_PIN4, LCD1602_DATA_PIN5, LCD1602_DATA_PIN6);
#endif /* LCD1602_NK */


#if !defined(SERIAL_INPUT)
int gnPosLX, gnPosLY, gnPosRX, gnPosRY;
int gnPosLAdjX, gnPosLAdjY, gnPosRAdjX, gnPosRAdjY;

int gnPosLX_Prev, gnPosLY_Prev, gnPosRX_Prev, gnPosRY_Prev;
int gnPosLAdjX_Prev, gnPosLAdjY_Prev, gnPosRAdjX_Prev, gnPosRAdjY_Prev;
#endif
uint8_t ucFlag_Prev;

uint8_t gucInitialized = 0;


//GPIOA.14

static u8 rf_setup;
static u8 proto_state;
static u32 bind_time;

#define PROTO_DEINIT    0x00
#define PROTO_INIT      0x01
#define PROTO_READY     0x02
#define PROTO_BINDING   0x04
#define PROTO_BINDDLG   0x08
#define PROTO_MODULEDLG 0x10


#ifdef SYMA_X5C_NEW_MODEL
void PROTOCOL_Init(u8 force)
{
  proto_state = PROTO_INIT;
  proto_state |= PROTO_READY;

  SYMAX_Cmds(PROTOCMD_INIT);
}
#endif /* SYMA_X5C_NEW_MODEL */

int PROTOCOL_SticksMoved(int init)
{
    const int32_t STICK_MOVEMENT = 15;   // defines when the bind dialog should be interrupted (stick movement STICK_MOVEMENT %)
    static int32_t ele_start, ail_start;
    int32_t ele;
    int32_t ail;
    
    ele = 0;  /* NK, power-off state (0), max = 255 */
    ail = 0;  /* NK, center/middle */

    if(init) {
        ele_start = ele;
        ail_start = ail;
        return 0;
    }
    int32_t ele_diff = abs(ele_start - ele);
    int32_t ail_diff = abs(ail_start - ail);
    return ((ele_diff + ail_diff > 2 * STICK_MOVEMENT * CHAN_MAX_VALUE / 100));
}

u32 PROTOCOL_Binding()
{

    if (proto_state & PROTO_BINDING) {
        if (bind_time == 0xFFFFFFFF)
            return bind_time;
#if 0
        int32_t tmp = bind_time - CLOCK_getms();
#else
        int32_t tmp = bind_time - millis();  // remaining time
#endif
        return tmp > 0 ? tmp : 1;
    }
    return 0;
}

void PROTOCOL_SetBindState(u32 msec)
{
    if (msec) {
        if (msec == 0xFFFFFFFF)
            bind_time = msec;
        else {
#if 0 
            bind_time = CLOCK_getms() + msec;
#else
            bind_time = millis() + msec;
#endif
        }
        proto_state |= PROTO_BINDING;
        PROTOCOL_SticksMoved(1);  //Initialize Stick position, NK - Looks not necessary
    } else {
        proto_state &= ~PROTO_BINDING;
    }
}


void setup() {
  int i, j, nRet;

#ifdef EXT_BUTTON
  eRX_DATA_RET_t eRxDataRet;
#endif /* EXT_BUTTON */
 
#ifdef LCD1602_NK
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("hello, world!");
#endif /* LCD1602_NK */
 
  // put your setup code here, to run once:
#ifdef EXT_BUTTON
  Serial.begin(9600);  /* Needs to be kept to communicate with External button module correctly */
#else
  Serial.begin(115200); /* Debug Console */
#endif /* EXT_BUTTON */
  printf_begin();

#if defined(NKD_DEBUG)
  printf("%7lu) ## NK [%s:%d] Syma X5C Setup\n", millis(), __func__, __LINE__);
#endif

#if defined(SYMA_X5C_LED)
  // set the digital pin as output:
  pinMode(gnLED1Pin, OUTPUT);
  digitalWrite(gnLED1Pin, 0);  // Default to low

  pinMode(gnLED2Pin, OUTPUT);
  digitalWrite(gnLED2Pin, 0);  // Default to low
#elif defined(NK_DEBUG_LED)
  pinMode(gnDebugLEDPin, OUTPUT);
  digitalWrite(gnDebugLEDPin, 0);  // Default to low    
#elif defined(RAINBOW_LED)
  // set the digital pin as output:
  pinMode(gnRainbowLEDPin, OUTPUT);
  digitalWrite(gnRainbowLEDPin, 0);  // Default to low
#endif

#ifdef MOTOR_PWM
  pinMode(gucMotorPwm_Pin1, OUTPUT);
  pinMode(gucMotorPwm_Pin2, OUTPUT);
  pinMode(gucMotorPwm_Pin3, OUTPUT);
  pinMode(gucMotorPwm_Pin4, OUTPUT);

  analogWrite(gucMotorPwm_Pin1, 0);
  analogWrite(gucMotorPwm_Pin2, 0);
  analogWrite(gucMotorPwm_Pin3, 0);
  analogWrite(gucMotorPwm_Pin4, 0);

  analogWrite(gucMotorPwm_Pin1, 10);  /* 0 ~ 255 */
  delay(200);  analogWrite(gucMotorPwm_Pin1, 0);  

  analogWrite(gucMotorPwm_Pin2, 10);  /* */
  delay(200);  analogWrite(gucMotorPwm_Pin2, 0);  
  
  analogWrite(gucMotorPwm_Pin3, 10);
  delay(200);  analogWrite(gucMotorPwm_Pin3, 0);  

  analogWrite(gucMotorPwm_Pin4, 10);  
  delay(200);  analogWrite(gucMotorPwm_Pin4, 0);
#endif /* MOTOR_PWM */

#ifdef MOTOR_L298_NK
  motor_l298_setup();
#endif

#ifdef SERVO_WHEEL_NK
  servo_wheel_setup();
#endif

  pinMode(gnNRF24L01_CEPin, OUTPUT);
  pinMode(gnNRF24L01_CSNPin, OUTPUT);  

#ifdef JOY_BUTTON
  pinMode(gnM2L_ButtonPin, INPUT);
  digitalWrite(gnM2L_ButtonPin, 1);
  
  pinMode(gnM2R_ButtonPin, INPUT);
  digitalWrite(gnM2R_ButtonPin, 1);
#endif /* JOY_BUTTON */

  // Initialize SPI class
  SPI.begin();

#ifdef PROTOCOL_SYMA_X5C_TRANSMITTER_MODE
  pinMode(gnM2L_xPin, INPUT);
  pinMode(gnM2L_yPin, INPUT);
  pinMode(gnM2R_xPin, INPUT);
  pinMode(gnM2R_yPin, INPUT);
#endif /* PROTOCOL_SYMA_X5C_TRANSMITTER_MODE */

#if defined(V2X2_MODEL) && !defined(SERIAL_INPUT)
  Calibrate_Throttle();
#endif

#ifdef MPU6050_INPUT
  mpu6050_setup();
#endif

#ifdef MELODY_NK
  MUSIC_Play(MELODY_READY);
#endif

#ifdef RAINBOW_LED
  pixels.begin();      // 라이브러리를 초기화 합니다.

//  RainbowLED_Test();
  RainbowLED_DebugLight(RB_LED_POS_BOOT_UP_PROGRESS, RB_LED_BOOT_UP_PROGRESS_1);
  RainbowLED_DebugLight(RB_LED_POS_BOOT_UP_PROGRESS, RB_LED_BOOT_UP_PROGRESS_2);
  RainbowLED_DebugLight(RB_LED_POS_BOOT_UP_PROGRESS, RB_LED_BOOT_UP_PROGRESS_3);
  RainbowLED_DebugLight(RB_LED_POS_BOOT_UP_PROGRESS, RB_LED_BOOT_UP_PROGRESS_4);

  RainbowLED_DebugLight(RB_LED_POS_DATA_PKT, RB_LED_DATA_PKT_MSG_VALID_DATA);
  RainbowLED_DebugLight(RB_LED_POS_DATA_PKT, RB_LED_DATA_PKT_MSG_INVALID_DATA);
  RainbowLED_DebugLight(RB_LED_POS_DATA_PKT, RB_LED_DATA_PKT_MSG_NO_DATA);
//  RainbowLED_DebugLight(RB_LED_POS_DATA_PKT, RB_LED_DATA_PKT_MSG3);

  RainbowLED_DebugLight(RB_LED_POS_MSG2, RB_LED_MSG2_1); /* Blue */
  RainbowLED_DebugLight(RB_LED_POS_MSG2, RB_LED_MSG2_2); /* Red */
  RainbowLED_DebugLight(RB_LED_POS_MSG2, RB_LED_MSG2_3); /* Green */
//  RainbowLED_DebugLight(RB_LED_POS_MSG2, RB_LED_MSG2_4); /* Off */

  RainbowLED_DebugLight(RB_LED_POS_MSG3, RB_LED_MSG3_1); /* Blue */
  RainbowLED_DebugLight(RB_LED_POS_MSG3, RB_LED_MSG3_2); /* Red */
  RainbowLED_DebugLight(RB_LED_POS_MSG3, RB_LED_MSG3_3); /* Green */
//  RainbowLED_DebugLight(RB_LED_POS_MSG3, RB_LED_MSG3_4); /* Off */
#endif /* RAINBOW_LED */

#ifdef EXT_BUTTON
#if defined(NKD_DEBUG)
  printf("%7lu) ## NK [%s:%d] ################################################\n", millis(), __func__, __LINE__);
  printf("%7lu) ## NK [%s:%d] sizeof(unsigned int) : %d\n", millis(), __func__, __LINE__, sizeof(unsigned int));
  printf("%7lu) ## NK [%s:%d] sizeof(struct TrimData) : %d\n", millis(), __func__, __LINE__, sizeof(struct TrimData));
  printf("%7lu) ## NK [%s:%d] sizeof(struct TrimEEPROMData) : %d\n", millis(), __func__, __LINE__, sizeof(struct TrimEEPROMData));
  printf("%7lu) ## NK [%s:%d] sizeof(struct TxData) : %d\n", millis(), __func__, __LINE__, sizeof(struct TxData));
  printf("%7lu) ## NK [%s:%d] DATA_PKT_MAGIC & 0xFF : 0x%02x\n", millis(), __func__, __LINE__, DATA_PKT_MAGIC & 0xFF);
  printf("%7lu) ## NK [%s:%d] ################################################\n", millis(), __func__, __LINE__,);
#endif

#ifdef RAINBOW_LED
  RainbowLED_DebugLight(RB_LED_POS_BOOT_UP_PROGRESS, RB_LED_BOOT_UP_PROGRESS_3);
#endif  

  unsigned long waiting_ext_btn_ready_at = millis();
  while( 1 ) {
  	if( millis() - waiting_ext_btn_ready_at > EXT_BTN_READY_TIMEOUT ) {
#ifdef MELODY_NK
	  MUSIC_Play(MELODY_FAILS);
#endif
#ifdef RAINBOW_LED
    RainbowLED_DebugLight(RB_LED_POS_DATA_PKT, RB_LED_DATA_PKT_MSG_INVALID_DATA);
#endif
      break;
	  }
    eRxDataRet = RX_DataPkt();
    if( eRxDataRet >= RX_DATA_VALID ) {
      gstRxPacketData = gstTempRxPacketData; // Copy the valid data to the static variable.
#ifdef RAINBOW_LED      
      RainbowLED_DebugLight(RB_LED_POS_DATA_PKT, RB_LED_DATA_PKT_MSG_VALID_DATA);
#endif
#ifdef MELODY_NK
      MUSIC_Play(MELODY_SUCCESS);
#endif
      break;  // Received a valid data packet
    }
  }
#endif /* EXT_BUTTON */

#ifdef CONFIG_HMC5883L_GY273_TRANSMITTER_NK

  if( sizeof (struct HMC5883L_AirData) != (PACKET_SIZE) ) {
#if defined(NKD_DEBUG)
    printf("%7lu) ## NK [%s:%d] ### Need to adjust the HMC5883L_AirData size to be same as 'PACKET_SIZE'\n", millis(), __func__, __LINE__);
#endif
    while(1);
  }

  // Initialize Initialize HMC5883L
#if defined(NKD_DEBUG)
  printf("%7lu) ## NK [%s:%d] Initialize HMC5883L\n", millis(), __func__, __LINE__);
#endif
  while (!compass.begin())
  {
#if defined(NKD_DEBUG)
    printf("%7lu) ## NK [%s:%d] Couldn't find a valid HMC5883L sensor, check wiring!!\n", millis(), __func__, __LINE__ );
#endif
    delay(500);
  }

  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino
//  compass.setOffset(351, -87); // NK
  compass.setOffset(457, -160); // NK

#endif /* CONFIG_HMC5883L_GY273_TRANSMITTER_NK */
} /* setup() */

void loop() {

  uint8_t i;

#if defined(NKD_FUNC)
  printf("%7lu) ## NK [%s:%d] [+]\n", millis(), __func__, __LINE__);
#endif

#if defined(CONFIG_SYMA_X5C_RECEIVER_NK) || defined(CONFIG_SYMA_X5C_RECEIVER_CAR2WD_NK)

  // NK - SYMAX_Cmds() are expected to return in receiver mode
  SYMAX_Cmds(PROTOCMD_INIT);

#else

  // put your main code here, to run repeatedly:
  if( gucInitialized == 0 ) {

#ifdef RAINBOW_LED
    RainbowLED_DebugLight(RB_LED_POS_BOOT_UP_PROGRESS, RB_LED_BOOT_UP_PROGRESS_4);
#endif  
    
#ifdef SYMAX_MODEL
    SYMAX_Cmds(PROTOCMD_INIT);
#elif defined(V2X2_MODEL)
    initV2x2();
#endif
    gucInitialized = 1;
  }

#endif

#ifdef EXT_BUTTON
  geRxDataRet = RX_DataPkt();
#if defined(NKD_DEBUG)
  printf("%7lu) ## NK [%s:%d] geRxDataRet : %d\n", millis(), __func__, __LINE__, geRxDataRet);
#endif

  if( geRxDataRet >= RX_DATA_VALID ) {
    gstRxPacketData = gstTempRxPacketData; // Copy the valid data to the static variable.    
#ifdef RAINBOW_LED      
    RainbowLED_DebugLight(RB_LED_POS_DATA_PKT, RB_LED_DATA_PKT_MSG_VALID_DATA);
#endif    
  }
  else {
    switch( geRxDataRet ) {
      case RX_DATA_TIMEOUT :
#ifdef RAINBOW_LED            
        RainbowLED_DebugLight(RB_LED_POS_DATA_PKT, RB_LED_DATA_PKT_MSG_NO_DATA);
#endif        
        break;

      case RX_DATA_CRC_ERROR :
#ifdef RAINBOW_LED
        RainbowLED_DebugLight(RB_LED_POS_MSG2, RB_LED_MSG2_1);
        RainbowLED_DebugLight(RB_LED_POS_MSG2, RB_LED_MSG2_2);
        RainbowLED_DebugLight(RB_LED_POS_MSG2, RB_LED_MSG2_3);
        RainbowLED_DebugLight(RB_LED_POS_MSG2, RB_LED_MSG2_4);

        RainbowLED_DebugLight(RB_LED_POS_MSG3, RB_LED_MSG3_1);
        RainbowLED_DebugLight(RB_LED_POS_MSG3, RB_LED_MSG3_2);
        RainbowLED_DebugLight(RB_LED_POS_MSG3, RB_LED_MSG3_3);
        RainbowLED_DebugLight(RB_LED_POS_MSG3, RB_LED_MSG3_4);
#endif
        break;

      case RX_DATA_MINMAX_ERROR :
#ifdef RAINBOW_LED            
        RainbowLED_DebugLight(RB_LED_POS_DATA_PKT, RB_LED_DATA_PKT_MSG_INVALID_DATA);
#endif        
        break;

      default :
#ifdef RAINBOW_LED            
        RainbowLED_DebugLight(RB_LED_POS_DATA_PKT, RB_LED_DATA_PKT_MSG_INVALID_DATA);
#endif        
        break;
    }
  }
#endif  /* EXT_BUTTON */

#if defined(CONFIG_SYMA_X5C_RECEIVER_NK) || defined(CONFIG_SYMA_X5C_RECEIVER_CAR2WD_NK)
extern uint8_t SymaxX5CRxAirData[MAX_PACKET_SIZE];
extern uint8_t ucCurrentChan;
extern uint8_t ucSymaxReceivedDataReady;

#if defined(NKD_DEBUG_VERBOSE)
  printf("%7lu) ## NK [%s:%d] [CH:%d, DataValid:%s] - ", millis(), __func__, __LINE__, ucCurrentChan, ucSymaxReceivedDataReady ? "O" : "X");
  for( i = 0; i < 6; i++) {
    printf("0x%02x ", SymaxX5CRxAirData[i]);
  }
  printf("\n");
#endif

  if( ucSymaxReceivedDataReady ) {

#ifdef MOTOR_L298_NK
    motor_l298_update();
#endif

#ifdef SERVO_WHEEL_NK
    servo_wheel_update();
#endif

    // Handle the data
    ucSymaxReceivedDataReady = 0;
  }
#endif

#ifdef CONFIG_HMC5883L_GY273_TRANSMITTER_NK

  Vector norm = compass.readNormalize();

  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (-8 + (28.0 / 60.0)) / (180 / M_PI); // NK - Seoul, magnet declination 
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  // Convert to degrees
  float headingDegrees = heading * 180/M_PI;

  // Output
#if defined(NKD_DEBUG_VERBOSE)
  printf("%7lu) ## NK [%s:%d] Heading = %d\n", millis(), __func__, __LINE__, headingDegrees);
#endif
  /* 
   * The Compass data is copied to the Symax packet buffer in the callback. It should be atomic. noInterrupts()/interrupts()
   */
  noInterrupts();
  gstEcompassTxAirPacketData.stEcompassAirData.unMagic = ECOMPASS_AIR_DATA_PKT_MAGIC;  
  gstEcompassTxAirPacketData.stEcompassAirData.gfCompassHeading = headingDegrees;
  gstEcompassTxAirPacketData.stEcompassAirData.ucOpt1 = millis() % 255;
//  gstEcompassTxAirPacketData.stEcompassAirData.ucChecksum = 0;  /* Will be calculated and set before sending the data packt via Air */
  interrupts();

#endif /* CONFIG_HMC5883L_GY273_TRANSMITTER_NK */

}
