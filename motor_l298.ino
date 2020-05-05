#ifdef MOTOR_L298_NK


#define BABS(X) (((X) < 0) ? -(u8)(X) : (X))

extern uint8_t SymaxX5CRxAirData[MAX_PACKET_SIZE];

void motor_l298_setup()
{
  pinMode(MOTOR_L298_ENA1_PIN, OUTPUT);
  pinMode(MOTOR_L298_IN1_PIN, OUTPUT);
  pinMode(MOTOR_L298_IN2_PIN, OUTPUT);
}

void motor_l298_set_direction_forward()
{
  digitalWrite(MOTOR_L298_IN1_PIN, HIGH);
  digitalWrite(MOTOR_L298_IN2_PIN, LOW);
}

void motor_l298_set_direction_backward()
{
  digitalWrite(MOTOR_L298_IN1_PIN, LOW);
  digitalWrite(MOTOR_L298_IN2_PIN, HIGH);
}

void motor_l298_set_speed(uint8_t motorSpeed)
{
#if defined(NKD_DEBUG_VERBOSE)
  printf("%7lu) ## NK [%s:%d] Motor Speed : %d\n", millis(), __func__, __LINE__, motorSpeed);
#endif
  analogWrite(MOTOR_L298_ENA1_PIN, motorSpeed); // Send PWM signal to motor A
}

void motor_l298_update()
{
  uint8_t ucTempMotorSpeed;

  if( !(SymaxX5CRxAirData[PKT_IDX_ELEVATOR] & 0x80) ) {

    motor_l298_set_direction_forward();

    ucTempMotorSpeed = SymaxX5CRxAirData[PKT_IDX_ELEVATOR];
  } else {

    motor_l298_set_direction_backward();

    ucTempMotorSpeed = BABS(SymaxX5CRxAirData[PKT_IDX_ELEVATOR]); /* -1 (0xFF) ~ -128 */
    ucTempMotorSpeed = ucTempMotorSpeed - 128;
  }

  ucTempMotorSpeed = map(ucTempMotorSpeed, 0, 127, 0, 255);

  motor_l298_set_speed(ucTempMotorSpeed);
}

#endif /* MOTOR_L298_NK */
