
//#define CONFIG_V2X2_TRANSMITTER_NK
//#define CONFIG_SYMA_X5C_TRANSMITTER_ELEPHANT_NK
//#define CONFIG_SYMA_X5C_TRANSMITTER_PLUS_NK
//#define CONFIG_SYMA_X5C_TRANSMITTER_NK
//#define CONFIG_SYMA_X5C_RECEIVER_NK
#define CONFIG_SYMA_X5C_RECEIVER_CAR2WD_NK
//define CONFIG_HMC5883L_GY273_TRANSMITTER_NK
//#define CONFIG_HMC5883L_GY273_RECEIVER_NK

//#define NK_DEBUG_LED
//#define RAINBOW_LED
//#define SYMA_X5C_LED

#define NRF24L01_DRIVER

/***************************************************
 *
 * Debug MASK Data structure / Macro       
 *
 ***************************************************/
//#define NKD_SPOT  // Sporatic message
//#define NKD_INFO  // General information
//#define NKD_DEBUG // Debugging message
//#define NKD_DEBUG_VERBOSE // Verbose debugging message
//#define NKD_DELTA // Debugging message for tracking difference(delta)
//#define NKD_FUNC


/************************************************************
 * CONFIG_V2X2_TRANSMITTER_NK
 * Tested on "Elephant Transmitter"
 ************************************************************/
#ifdef CONFIG_V2X2_TRANSMITTER_NK

#include "TimerOne.h"
//#define SYMAX_MODEL
//#define SYMA_X5C_NEW_MODEL
//#define PROTOCOL_SYMA_X5C_MODEL
//#define PROTOCOL_SYMA_X5C_TRANSMITTER_MODE
#define V2X2_MODEL
//#define SERIAL_INPUT
#define AETR  // Default:ATER
#define THROTTLE_CALIBRATION

#define NRF24L01_TRANSMITTER_MODE
#define NRF24L01_EXPERIMENT_FEATURE
//#define NRF24L01_TRANSMITTER_USE_FIXED_RXTX_ADDR

#define NK_DEBUG_TIMER_MONITOR

#define GIMBAL_JOYSTICK_INPUT  // Transmitter has joystick input(gimbal)
#define GIMBAL_JOYSTICK_LEFT_X_PIN  (A0)
#define GIMBAL_JOYSTICK_LEFT_Y_PIN  (A1)
#define GIMBAL_JOYSTICK_RIGHT_X_PIN (A2)
#define GIMBAL_JOYSTICK_RIGHT_Y_PIN (A3)

#define HW_SPI
//#define SW_SPI
//#define SW_SPI_NRF24L01_TRANSMITTER
//#define SW_SPI_MULTIPROTOCOL4IN1

//#define MPU6050_INPUT
#define MELODY_NK

//#define JOY_BUTTON
//#define GIMBAL_JOYSTICK_BUTTON_LEFT_PIN   (7)
//#define GIMBAL_JOYSTICK_BUTTON_RIGHT_PIN  (8)
//#define EXT_BUTTON

//#define NK_DEBUG_LED
//#define NK_DEBUG_LED_PIN  (4)
#define RAINBOW_LED
#define RAINBOW_LED_PIN   (6)
//#define SYMA_X5C_LED

#define MELODY_PIN        (3)
#define NRF24L01_CE_PIN   (9)
#define NRF24L01_CSN_PIN  (10)

#endif /* CONFIG_V2X2_TRANSMITTER_NK */

/************************************************************
 * CONFIG_SYMA_X5C_TRANSMITTER_ELEPHANT_NK
 ************************************************************/
#ifdef CONFIG_SYMA_X5C_TRANSMITTER_ELEPHANT_NK

#include "TimerOne.h"
#define SYMAX_MODEL
//#define SYMA_X5C_NEW_MODEL
#define PROTOCOL_SYMA_X5C_MODEL
#define PROTOCOL_SYMA_X5C_TRANSMITTER_MODE

#define NRF24L01_TRANSMITTER_MODE

//#define NRF24L01_TRANSMITTER_CHANNEL_SYMA_X5C_DEVICE
#define NRF24L01_TRANSMITTER_CHANNEL_IN_SYMA_X5C_SOURCE_CODE
//#define NRF24L01_TRANSMITTER_CHANNEL_EXPERIMENT

#define NRF24L01_EXPERIMENT_FEATURE
//#define NRF24L01_TRANSMITTER_USE_FIXED_RXTX_ADDR
//#define NRF24L01_USE_FIXED_CHANNELS

#define NK_DEBUG_TIMER_MONITOR

#define GIMBAL_JOYSTICK_INPUT  // Transmitter has joystick input(gimbal)
#define GIMBAL_JOYSTICK_LEFT_X_PIN  (A0)
#define GIMBAL_JOYSTICK_LEFT_Y_PIN  (A1)
#define GIMBAL_JOYSTICK_RIGHT_X_PIN (A2)
#define GIMBAL_JOYSTICK_RIGHT_Y_PIN (A3)

#define HW_SPI
//#define SW_SPI
//#define SW_SPI_NRF24L01_TRANSMITTER
//#define SW_SPI_MULTIPROTOCOL4IN1

//#define MPU6050_INPUT
#define MELODY_NK

//#define JOY_BUTTON
//#define GIMBAL_JOYSTICK_BUTTON_LEFT_PIN   (7)
//#define GIMBAL_JOYSTICK_BUTTON_RIGHT_PIN  (8)
//#define EXT_BUTTON

//#define NK_DEBUG_LED
//#define NK_DEBUG_LED_PIN  (4)
//#define RAINBOW_LED
//#define RAINBOW_LED_PIN   (6)
//#define SYMA_X5C_LED

#define MELODY_PIN        (4)
#define NRF24L01_CE_PIN   (9)
#define NRF24L01_CSN_PIN  (10)

#endif /* CONFIG_SYMA_X5C_TRANSMITTER_ELEPHANT_NK */

/************************************************************
 * CONFIG_SYMA_X5C_TRANSMITTER_PLUS_NK
 ************************************************************/
#ifdef CONFIG_SYMA_X5C_TRANSMITTER_PLUS_NK

#include "TimerOne.h"
#define SYMAX_MODEL
//#define SYMA_X5C_NEW_MODEL
#define PROTOCOL_SYMA_X5C_MODEL
#define PROTOCOL_SYMA_X5C_TRANSMITTER_MODE

#define NRF24L01_TRANSMITTER_MODE
#define NRF24L01_EXPERIMENT_FEATURE
#define NRF24L01_TRANSMITTER_USE_FIXED_RXTX_ADDR

#define NK_DEBUG_TIMER_MONITOR

#define GIMBAL_JOYSTICK_INPUT  // Transmitter has joystick input(gimbal)
#define GIMBAL_JOYSTICK_LEFT_X_PIN  (A0)
#define GIMBAL_JOYSTICK_LEFT_Y_PIN  (A1)
#define GIMBAL_JOYSTICK_RIGHT_X_PIN (A2)
#define GIMBAL_JOYSTICK_RIGHT_Y_PIN (A3)

#define HW_SPI
//#define SW_SPI
//#define SW_SPI_NRF24L01_TRANSMITTER
//#define SW_SPI_MULTIPROTOCOL4IN1

//#define MPU6050_INPUT
#define MELODY_NK

//#define JOY_BUTTON
//#define GIMBAL_JOYSTICK_BUTTON_LEFT_PIN   (7)
//#define GIMBAL_JOYSTICK_BUTTON_RIGHT_PIN  (8)
//#define EXT_BUTTON

//#define NK_DEBUG_LED
//#define NK_DEBUG_LED_PIN  (4)
#define RAINBOW_LED
#define RAINBOW_LED_PIN   (6)
//#define SYMA_X5C_LED

#define MELODY_PIN        (3)
#define NRF24L01_CE_PIN   (9)
#define NRF24L01_CSN_PIN  (10)

#endif /* CONFIG_SYMA_X5C_TRANSMITTER_PLUS_NK */


/************************************************************
 * CONFIG_SYMA_X5C_TRANSMITTER_NK
 ************************************************************/
#ifdef CONFIG_SYMA_X5C_TRANSMITTER_NK

#include "TimerOne.h"
#define SYMAX_MODEL
//#define SYMA_X5C_NEW_MODEL
#define PROTOCOL_SYMA_X5C_MODEL
#define PROTOCOL_SYMA_X5C_TRANSMITTER_MODE

#define NRF24L01_TRANSMITTER_MODE

/* Choose one out of 3 below */
//#define NRF24L01_TRANSMITTER_CHANNEL_SYMA_X5C_DEVICE
#define NRF24L01_TRANSMITTER_CHANNEL_IN_SYMA_X5C_SOURCE_CODE
//#define NRF24L01_TRANSMITTER_CHANNEL_EXPERIMENT

#define NRF24L01_EXPERIMENT_FEATURE
//#define NRF24L01_TRANSMITTER_USE_FIXED_RXTX_ADDR
//#define NRF24L01_USE_FIXED_CHANNELS

#define NK_DEBUG_TIMER_MONITOR

#define GIMBAL_JOYSTICK_INPUT  // Transmitter has joystick input(gimbal)
#define GIMBAL_JOYSTICK_LEFT_X_PIN  (A0)
#define GIMBAL_JOYSTICK_LEFT_Y_PIN  (A1)
#define GIMBAL_JOYSTICK_RIGHT_X_PIN (A2)
#define GIMBAL_JOYSTICK_RIGHT_Y_PIN (A3)

#define HW_SPI
//#define SW_SPI
//#define SW_SPI_NRF24L01_TRANSMITTER
//#define SW_SPI_MULTIPROTOCOL4IN1

//#define MPU6050_INPUT
#define MELODY_NK

//#define JOY_BUTTON
//#define GIMBAL_JOYSTICK_BUTTON_LEFT_PIN   (7)
//#define GIMBAL_JOYSTICK_BUTTON_RIGHT_PIN  (8)
//#define EXT_BUTTON	// It causes long delay in the loop

//#define NK_DEBUG_LED
//#define NK_DEBUG_LED_PIN  (4)
//#define RAINBOW_LED
//#define RAINBOW_LED_PIN   (6)
//#define SYMA_X5C_LED

#define MELODY_PIN        (3)
#define NRF24L01_CE_PIN   (9)
#define NRF24L01_CSN_PIN  (10)

#endif /* CONFIG_SYMA_X5C_TRANSMITTER_NK */

/************************************************************
 * CONFIG_SYMA_X5C_RECEIVER_NK
 ************************************************************/
#ifdef CONFIG_SYMA_X5C_RECEIVER_NK

#define SYMAX_MODEL
//#define SYMA_X5C_NEW_MODEL
#define PROTOCOL_SYMA_X5C_MODEL
#define PROTOCOL_SYMA_X5C_RECEIVER_MODE

#define NRF24L01_RECEIVER_MODE
//#define NRF24L01_RECEIVER_CHANNEL_BALANCE_CHECK_NK

/* Choose one out of 3 below */
#define NRF24L01_RECEIVER_CHANNEL_SYMA_X5C_DEVICE
//#define NRF24L01_RECEIVER_CHANNEL_IN_SYMA_X5C_SOURCE_CODE
//#define NRF24L01_RECEIVER_CHANNEL_EXPERIMENT

#define NRF24L01_EXPERIMENT_FEATURE // register dump/summary
//#define NRF24L01_USE_FIXED_CHANNELS

#define NK_DEBUG_TIMER_MONITOR

#define HW_SPI
//#define SW_SPI
//#define SW_SPI_NRF24L01_TRANSMITTER
//#define SW_SPI_MULTIPROTOCOL4IN1

#define MOTOR_PWM
#define MOTOR_PIN1        (5)
#define MOTOR_PIN2        (3)
#define MOTOR_PIN3        (9)
#define MOTOR_PIN4        (6)

#define NK_RX_PKT_STAT
#define NK_RX_PKT_STAT_CYCLE  (10000) // every 10 sec (10000 ms).

//#define MPU6050_INPUT
//#define MELODY_NK

//#define JOY_BUTTON
//#define EXT_BUTTON

#define NRF24L01_CE_PIN   (7)
#define NRF24L01_CSN_PIN  (8)

#endif /* CONFIG_SYMA_X5C_RECEIVER_NK */

/************************************************************
 * CONFIG_SYMA_X5C_RECEIVER_CAR2WD_NK
 ************************************************************/
#ifdef CONFIG_SYMA_X5C_RECEIVER_CAR2WD_NK

#define SYMAX_MODEL
//#define SYMA_X5C_NEW_MODEL
#define PROTOCOL_SYMA_X5C_MODEL
#define PROTOCOL_SYMA_X5C_RECEIVER_MODE

#define NRF24L01_RECEIVER_MODE
//#define NRF24L01_RECEIVER_CHANNEL_BALANCE_CHECK_NK

/* Choose one out of 3 below */
#define NRF24L01_RECEIVER_CHANNEL_SYMA_X5C_DEVICE
//#define NRF24L01_RECEIVER_CHANNEL_IN_SYMA_X5C_SOURCE_CODE
//#define NRF24L01_RECEIVER_CHANNEL_EXPERIMENT

#define NRF24L01_EXPERIMENT_FEATURE // register dump/summary
//#define NRF24L01_USE_FIXED_CHANNELS

#define SERVO_WHEEL_NK
#define SERVO_WHEEL_PIN1    (9)

//#define HW_SPI
#define SW_SPI
//#define SW_SPI_NRF24L01_TRANSMITTER
#define SW_SPI_MULTIPROTOCOL4IN1

//#define MOTOR_PWM
#define MOTOR_L298_NK
#ifdef SW_SPI
#define MOTOR_L298_ENA1_PIN (10)
#define MOTOR_L298_IN1_PIN  (2)
#define MOTOR_L298_IN2_PIN  (3)
#else
#define MOTOR_L298_ENA1_PIN (6)
#define MOTOR_L298_IN1_PIN  (4)
#define MOTOR_L298_IN2_PIN  (5)
#endif

#define NK_RX_PKT_STAT
#define NK_RX_PKT_STAT_CYCLE  (10000) // every 10 sec (10000 ms).

//#define MPU6050_INPUT
//#define MELODY_NK

//#define JOY_BUTTON
//#define EXT_BUTTON

//#define NK_DEBUG_LED
//#define NK_DEBUG_LED_PIN  (4)
#define RAINBOW_LED
#define RAINBOW_LED_PIN   (11)
//#define SYMA_X5C_LED

#define NRF24L01_CE_PIN   (7)
#define NRF24L01_CSN_PIN  (8)

#endif /* CONFIG_SYMA_X5C_RECEIVER_CAR2WD_NK */

/************************************************************
 * CONFIG_HMC5883L_GY273_TRANSMITTER_NK
 ************************************************************/
#ifdef CONFIG_HMC5883L_GY273_TRANSMITTER_NK
#include "TimerOne.h"
#define SYMAX_MODEL
//#define SYMA_X5C_NEW_MODEL
#define PROTOCOL_SYMA_X5C_MODEL
#define PROTOCOL_SYMA_X5C_TRANSMITTER_MODE

#define NRF24L01_TRANSMITTER_MODE
#define NRF24L01_EXPERIMENT_FEATURE

#define NK_DEBUG_TIMER_MONITOR

#define GIMBAL_JOYSTICK_INPUT  // Transmitter has joystick input(gimbal)
#define GIMBAL_JOYSTICK_LEFT_X_PIN  (A0)
#define GIMBAL_JOYSTICK_LEFT_Y_PIN  (A1)
#define GIMBAL_JOYSTICK_RIGHT_X_PIN (A2)
#define GIMBAL_JOYSTICK_RIGHT_Y_PIN (A3)

#define HW_SPI
//#define SW_SPI
//#define SW_SPI_NRF24L01_TRANSMITTER
//#define SW_SPI_MULTIPROTOCOL4IN1

//#define MPU6050_INPUT
//#define MELODY_NK

//#define JOY_BUTTON
//#define EXT_BUTTON

#define NRF24L01_CE_PIN   (9)
#define NRF24L01_CSN_PIN  (10)

#endif /* CONFIG_HMC5883L_GY273_TRANSMITTER_NK */

/************************************************************
 * CONFIG_HMC5883L_GY273_RECEIVER_NK
 ************************************************************/
#ifdef CONFIG_HMC5883L_GY273_RECEIVER_NK
#define SYMAX_MODEL
//#define SYMA_X5C_NEW_MODEL
#define PROTOCOL_SYMA_X5C_MODEL
#define PROTOCOL_SYMA_X5C_RECEIVER_MODE

#define NRF24L01_RECEIVER_MODE
//#define NRF24L01_RECEIVER_CHANNEL_BALANCE_CHECK_NK
#define NRF24L01_RECEIVER_CHANNEL_SYMA_X5C_DEVICE
#define NRF24L01_EXPERIMENT_FEATURE

#define HW_SPI
//#define SW_SPI
//#define SW_SPI_NRF24L01_TRANSMITTER
//#define SW_SPI_MULTIPROTOCOL4IN1

//#define MOTOR_PWM
#define NK_RX_PKT_STAT
#define NK_RX_PKT_STAT_CYCLE  (10000) // every 10 sec (10000 ms).

//#define MPU6050_INPUT
//#define MELODY_NK

#define LCD1602_NK
#define LCD1602_DATA_PIN1 (10)
#define LCD1602_DATA_PIN2 (9)
#define LCD1602_DATA_PIN3 (6)
#define LCD1602_DATA_PIN4 (5)
#define LCD1602_DATA_PIN5 (4)
#define LCD1602_DATA_PIN6 (3)


//#define JOY_BUTTON
//#define EXT_BUTTON

#define NRF24L01_CE_PIN   (7)
#define NRF24L01_CSN_PIN  (8)

#endif /* CONFIG_HMC5883L_GY273_RECEIVER_NK */
