#ifdef RAINBOW_LED

/***************************************************
 * 
 * External function
 *
 ***************************************************/

extern void RainbowLED_DebugLight(uint8_t ucLEDPos, uint8_t ucMsgId);


/***************************************************
 *
 * Module : RainbowLED
 * Description : Index of the LED and the color of the LED implies the msg. 
 *
 ***************************************************/
typedef enum RB_LED_POS { 
  RB_LED_POS_BOOT_UP_PROGRESS = 0,
  RB_LED_POS_DATA_PKT = 1,
  RB_LED_POS_MSG2 = 2,
  RB_LED_POS_MSG3 = 3,
  RB_LED_POS_MSG_MAX,
} eRB_LED_POS_t;

typedef enum RB_LED_BOOT_UP_PROGRESS {
  RB_LED_BOOT_UP_PROGRESS_1 = 0,
  RB_LED_BOOT_UP_PROGRESS_2 = 1,
  RB_LED_BOOT_UP_PROGRESS_3 = 2,
  RB_LED_BOOT_UP_PROGRESS_4 = 3,
  RB_LED_BOOT_UP_PROGRESS_MAX,
} eRB_LED_BOOT_UP_PROGRESS_t;

typedef enum RB_LED_DATA_PKT_MSG { 
  RB_LED_DATA_PKT_MSG_VALID_DATA = 0,
  RB_LED_DATA_PKT_MSG_INVALID_DATA = 1,
  RB_LED_DATA_PKT_MSG_NO_DATA = 2,
  RB_LED_DATA_PKT_MSG3 = 3,
  RB_LED_DATA_PKT_MSG_MAX,
} eRB_LED_DATA_PKT_MSG_t;

typedef enum RB_LED_MSG2 { 
  RB_LED_MSG2_1 = 0,
  RB_LED_MSG2_2 = 1,
  RB_LED_MSG2_3 = 2,
  RB_LED_MSG2_4 = 3,  
  RB_LED_MSG2_MAX,
} eRB_LED_MSG2_t;

typedef enum RB_LED_MSG3 { 
  RB_LED_MSG3_1 = 0,
  RB_LED_MSG3_2 = 1,
  RB_LED_MSG3_3 = 2,
  RB_LED_MSG3_4 = 3,  
  RB_LED_MSG3_MAX,
} eRB_LED_MSG3_t;


typedef enum RB_LED_COLORS { 
  RB_LED_COLOR_RED = 0,
  RB_LED_COLOR_GREEN = 1,
  RB_LED_COLOR_BLUE = 2,
  RB_LED_COLOR_MAX,
} eRB_LED_COLORS_t;

#endif /* RAINBOW_LED */
