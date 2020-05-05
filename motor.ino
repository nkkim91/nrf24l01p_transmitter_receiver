#ifdef MOTOR_PWM

#define ADJUST_RUDDER_TO_RIGHT  (50)

static uint8_t  ucMotorPwm[NUM_MOTORS];
static int16_t  sMotorPwmAdjust[NUM_MOTORS];

extern uint8_t SymaxX5CRxAirData[MAX_PACKET_SIZE];

static void calculate_motor_pwm(void)
{
  uint8_t  i, ucAbsControlOffset;
  int16_t  sTmp[4];

  /* Throttle */
  for( i = 0; i < NUM_MOTORS; i++) {
    ucMotorPwm[i] = SymaxX5CRxAirData[PKT_IDX_THROTTLE];
    sMotorPwmAdjust[i] = 0;
  }

#if 0
  if( packet[PKT_IDX_THROTTLE] > 0 ) {
    if( ucMotorPwm[MOTOR_1_IDX] + ADJUST_RUDDER_TO_RIGHT > 255 ) {
      ucMotorPwm[MOTOR_1_IDX] = 255;
    } else {
      ucMotorPwm[MOTOR_1_IDX] += ADJUST_RUDDER_TO_RIGHT;
    }
    if( ucMotorPwm[MOTOR_3_IDX] + ADJUST_RUDDER_TO_RIGHT > 255 ) {
      ucMotorPwm[MOTOR_3_IDX] = 255;
    } else {
      ucMotorPwm[MOTOR_3_IDX] += ADJUST_RUDDER_TO_RIGHT;
    }
  }
#endif  

  /* Aileron */  
  if( (int8_t)SymaxX5CRxAirData[PKT_IDX_AILERON] < 0 ) {  /* Aileron : Right */
    ucAbsControlOffset = SymaxX5CRxAirData[PKT_IDX_AILERON] & ~0x80;
    sMotorPwmAdjust[MOTOR_3_IDX] += ucAbsControlOffset;
    sMotorPwmAdjust[MOTOR_4_IDX] += ucAbsControlOffset;

    sMotorPwmAdjust[MOTOR_1_IDX] -= ucAbsControlOffset;
    sMotorPwmAdjust[MOTOR_2_IDX] -= ucAbsControlOffset;

#if defined(NKD_DEBUG_VERBOSE)
    printf("%7lu) ## NK [%s:%d] Aileron  : (-) %d\n", millis(), __func__, __LINE__, ucAbsControlOffset);
#endif
  } else {
    ucAbsControlOffset = SymaxX5CRxAirData[PKT_IDX_AILERON];    
    sMotorPwmAdjust[MOTOR_1_IDX] += ucAbsControlOffset;
    sMotorPwmAdjust[MOTOR_2_IDX] += ucAbsControlOffset;

    sMotorPwmAdjust[MOTOR_3_IDX] -= ucAbsControlOffset;
    sMotorPwmAdjust[MOTOR_4_IDX] -= ucAbsControlOffset;

#if defined(NKD_DEBUG_VERBOSE)
    printf("%7lu) ## NK [%s:%d] Aileron  : (+) %d\n", millis(), __func__, __LINE__, ucAbsControlOffset);
#endif
  }

#if defined(NKD_DEBUG_VERBOSE)
  printf("%7lu) ## NK [%s:%d] Aileron : %d, sMotorPwmAdjust [%d %d %d %d]\n", millis(), __func__, __LINE__, ucAbsControlOffset, sMotorPwmAdjust[MOTOR_1_IDX], sMotorPwmAdjust[MOTOR_2_IDX], sMotorPwmAdjust[MOTOR_3_IDX], sMotorPwmAdjust[MOTOR_4_IDX]);
#endif

  /* Elevator */  
  if( (int8_t)SymaxX5CRxAirData[PKT_IDX_ELEVATOR] < 0 ) {  /* Elevator : Down */
    ucAbsControlOffset = SymaxX5CRxAirData[PKT_IDX_ELEVATOR] & ~0x80;
    sMotorPwmAdjust[MOTOR_1_IDX] += ucAbsControlOffset;
    sMotorPwmAdjust[MOTOR_4_IDX] += ucAbsControlOffset;
    
    sMotorPwmAdjust[MOTOR_2_IDX] -= ucAbsControlOffset;
    sMotorPwmAdjust[MOTOR_3_IDX] -= ucAbsControlOffset;

#if defined(NKD_DEBUG_VERBOSE)
    printf("%7lu) ## NK [%s:%d] Elevator : (-) %d\n", millis(), __func__, __LINE__, ucAbsControlOffset);
#endif
  } else {
    ucAbsControlOffset = SymaxX5CRxAirData[PKT_IDX_ELEVATOR];    
    sMotorPwmAdjust[MOTOR_2_IDX] += ucAbsControlOffset;
    sMotorPwmAdjust[MOTOR_3_IDX] += ucAbsControlOffset;

    sMotorPwmAdjust[MOTOR_1_IDX] -= ucAbsControlOffset;
    sMotorPwmAdjust[MOTOR_4_IDX] -= ucAbsControlOffset;
   
#if defined(NKD_DEBUG_VERBOSE)
    printf("%7lu) ## NK [%s:%d] Elevator : (+) %d\n", millis(), __func__, __LINE__, ucAbsControlOffset);
#endif
  }

#if defined(NKD_DEBUG_VERBOSE)
  printf("%7lu) ## NK [%s:%d] Elevator : %d, sMotorPwmAdjust [%d %d %d %d]\n", millis(), __func__, __LINE__, ucAbsControlOffset, sMotorPwmAdjust[MOTOR_1_IDX], sMotorPwmAdjust[MOTOR_2_IDX], sMotorPwmAdjust[MOTOR_3_IDX], sMotorPwmAdjust[MOTOR_4_IDX]);
#endif

  /* Rudder */  
  if( (int8_t)SymaxX5CRxAirData[PKT_IDX_RUDDER] < 0 ) {  /* Rudder : Right */
    ucAbsControlOffset = SymaxX5CRxAirData[PKT_IDX_RUDDER] & ~0x80;
    sMotorPwmAdjust[MOTOR_1_IDX] += ucAbsControlOffset;
    sMotorPwmAdjust[MOTOR_3_IDX] += ucAbsControlOffset;

    sMotorPwmAdjust[MOTOR_2_IDX] -= ucAbsControlOffset;
    sMotorPwmAdjust[MOTOR_4_IDX] -= ucAbsControlOffset;
    
#if defined(NKD_DEBUG_VERBOSE)
    printf("%7lu) ## NK [%s:%d] Rudder   : (-) %d\n", millis(), __func__, __LINE__, ucAbsControlOffset);
#endif
  } else {
    ucAbsControlOffset = SymaxX5CRxAirData[PKT_IDX_RUDDER];    
    sMotorPwmAdjust[MOTOR_2_IDX] += ucAbsControlOffset;
    sMotorPwmAdjust[MOTOR_4_IDX] += ucAbsControlOffset;

    sMotorPwmAdjust[MOTOR_1_IDX] -= ucAbsControlOffset;
    sMotorPwmAdjust[MOTOR_3_IDX] -= ucAbsControlOffset;
   
#if defined(NKD_DEBUG_VERBOSE)
    printf("%7lu) ## NK [%s:%d] Rudder   : (+) %d\n", millis(), __func__, __LINE__, ucAbsControlOffset);
#endif
  }
  
#if defined(NKD_DEBUG_VERBOSE)
  printf("%7lu) ## NK [%s:%d] Rudder : %d, sMotorPwmAdjust [%d %d %d %d]\n", millis(), __func__, __LINE__, ucAbsControlOffset, sMotorPwmAdjust[MOTOR_1_IDX], sMotorPwmAdjust[MOTOR_2_IDX], sMotorPwmAdjust[MOTOR_3_IDX], sMotorPwmAdjust[MOTOR_4_IDX]);
#endif

  sTmp[0] = map(sMotorPwmAdjust[MOTOR_1_IDX], -128*3, 128*3, -ucMotorPwm[MOTOR_1_IDX], ucMotorPwm[MOTOR_1_IDX]);
  if( ucMotorPwm[MOTOR_1_IDX] + sTmp[0] > 255 ) {
    ucMotorPwm[MOTOR_1_IDX] = 255;
  } else {
    ucMotorPwm[MOTOR_1_IDX] = ucMotorPwm[MOTOR_1_IDX] + sTmp[0];
  }
 
  sTmp[1] = map(sMotorPwmAdjust[MOTOR_2_IDX], -128*3, 128*3, -ucMotorPwm[MOTOR_2_IDX], ucMotorPwm[MOTOR_2_IDX]);
  if( ucMotorPwm[MOTOR_2_IDX] + sTmp[1] > 255 ) {
    ucMotorPwm[MOTOR_2_IDX] = 255;
  } else {
    ucMotorPwm[MOTOR_2_IDX] = ucMotorPwm[MOTOR_2_IDX] + sTmp[1];
  }

  sTmp[2] = map(sMotorPwmAdjust[MOTOR_3_IDX], -128*3, 128*3, -ucMotorPwm[MOTOR_3_IDX], ucMotorPwm[MOTOR_3_IDX]);
  if( ucMotorPwm[MOTOR_3_IDX] + sTmp[2] > 255 ) {
    ucMotorPwm[MOTOR_3_IDX] = 255;
  } else {
    ucMotorPwm[MOTOR_3_IDX] = ucMotorPwm[MOTOR_3_IDX] + sTmp[2];
  }

  sTmp[3] = map(sMotorPwmAdjust[MOTOR_4_IDX], -128*3, 128*3, -ucMotorPwm[MOTOR_4_IDX], ucMotorPwm[MOTOR_4_IDX]); 
  if( ucMotorPwm[MOTOR_4_IDX] + sTmp[3] > 255 ) {
    ucMotorPwm[MOTOR_4_IDX] = 255;
  } else {
    ucMotorPwm[MOTOR_4_IDX] = ucMotorPwm[MOTOR_4_IDX] + sTmp[3];
  }

#if defined(NKD_DEBUG)
  printf("%7lu) ## NK [%s:%d] %d %d %d %d / %d %d %d %d\n", millis(), __func__, __LINE__, sTmp[0], sTmp[1], sTmp[2], sTmp[3], ucMotorPwm[MOTOR_1_IDX], ucMotorPwm[MOTOR_2_IDX], ucMotorPwm[MOTOR_3_IDX], ucMotorPwm[MOTOR_4_IDX]);
#endif
}

const unsigned char adjust_pwm_pin5_6[] = { 0, 91, 97, 107, 114, 120, 
                128, 135, 143, 150, 159, 
                166, 171, 178, 185, 190, 
                200, 210, 220, 230, 240, 
                240, 240, 240, 240, 240 
};

#endif /* MOTOR_PWM */
