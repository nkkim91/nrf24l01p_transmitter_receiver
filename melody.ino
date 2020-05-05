#ifdef MELODY_NK

void MUSIC_Play(int mode)
{
    pinMode(gnMelodyPin, OUTPUT);
    switch(mode) {
      case MELODY_SUCCESS : 
        tone(gnMelodyPin, NOTE_C4, 125);
        delay(125*1.30);
        tone(gnMelodyPin, NOTE_G3, 62);
        delay(62*1.30);
        tone(gnMelodyPin, NOTE_G3, 62);
        delay(62*1.30);
        tone(gnMelodyPin, NOTE_A3, 125);
        delay(125*1.30);
        tone(gnMelodyPin, NOTE_G3, 125);
        delay(125*1.30);
        tone(gnMelodyPin, 0, 125);
        delay(125*1.30);
        tone(gnMelodyPin, NOTE_B3, 125);
        delay(125*1.30);
        tone(gnMelodyPin, NOTE_C4, 125);
        delay(125*1.30);
        break;
      case MELODY_READY : 
        tone(gnMelodyPin, NOTE_C7, 125);
        delay(125*1.30);
        tone(gnMelodyPin, NOTE_D7, 125);
        delay(125*1.30);
        tone(gnMelodyPin, NOTE_E7, 125);
        delay(125*1.30);
        break;
      case MELODY_FAILS :
        tone(gnMelodyPin, NOTE_D7, 125*2);
        delay(125 * 2);
        noTone(gnMelodyPin);
        delay(125 * 1);
        tone(gnMelodyPin, NOTE_D7, 125*1);
        delay(125 * 1);
        noTone(gnMelodyPin);
        delay(125 * 0.5);
        tone(gnMelodyPin, NOTE_D7, 125*1);
        delay(125 * 1);
        noTone(gnMelodyPin);
        delay(125 * 0.5);
        tone(gnMelodyPin, NOTE_D7, 125*4);
        delay(125 * 4);
        noTone(gnMelodyPin);
        delay(125 * 2);
        break;
      default : 
        tone(gnMelodyPin, NOTE_C7, 125);
        delay(125*1.30);
        break;
    }
    // stop the tone playing:
    noTone(gnMelodyPin);
    pinMode(gnMelodyPin, INPUT);    
}

#endif /* MELODY_NK */
