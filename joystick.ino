#ifdef JOY_BUTTON

int ReadKeyInput(u8 *state)
{
  int ret = 0;

  if( digitalRead(gnM2L_ButtonPin) == 0 ) {  /* Pressed */
      *state |= 0x01;  /* CAMERA */
      ret = 1;
  }
  else if( digitalRead(gnM2R_ButtonPin) == 0 ) {
      *state |= 0x02;  /* VIDEO */
      ret = 1;
  } else {
      *state &= ~0x01;
      ret = 1;
  }

  return ret;
}

#endif
