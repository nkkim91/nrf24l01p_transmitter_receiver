#ifdef SERVO_WHEEL_NK

#include <Servo.h>

Servo servo_front_wheel;



void servo_wheel_setup() {
  
  servo_front_wheel.attach(SERVO_WHEEL_PIN1);

  servo_front_wheel.write(90);  /* Middle */
}

void servo_wheel_update()
{
  uint8_t ucTempServoAngle, ucTempRudder;

  if( ucSymaxReceivedDataReady == 1 ) { // Valid data ready

    if( !(SymaxX5CRxAirData[PKT_IDX_RUDDER] & 0x80) ) {
      ucTempRudder = SymaxX5CRxAirData[PKT_IDX_RUDDER]; /* 127 ~ 0 => 60 ~ 90 */
      ucTempRudder = 127 - ucTempRudder;
    } else {
      ucTempRudder = BABS(SymaxX5CRxAirData[PKT_IDX_RUDDER]); /* -1 (0xFF) ~ -128 */
      ucTempRudder = ucTempRudder;
    }

    ucTempRudder = map(ucTempRudder, 0, 255, 60, 120);
  
    servo_front_wheel.write(ucTempRudder);
  } else {
	// do nothing when receivedd ata Is NOT ready
  }
}

#endif /* SERVO_WHEEL_NK */
