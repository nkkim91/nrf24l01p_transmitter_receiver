#ifdef RAINBOW_LED

/***************************************************
 *
 * Rainbow LED Data structure / Macro
 *
 ***************************************************/

#define NUMPIXELS       4  // Number of LEDs.
#define PIN             RAINBOW_LED_PIN  // Rainbow LED connected to Arduino digital PIN
#define GAP            90  // GAP btw. color
#define SPEED          50  // Moving speed
#define RAINBOWLED_DELAY         100  // Delay in btw. display

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

unsigned char gucColorRed;
unsigned char gucColorGreen;
unsigned char gucColorBlue;

uint8_t gucRBLEDCurrentColor[RB_LED_POS_MSG_MAX][RB_LED_COLOR_MAX] = { 
        { 0, 0, 0 },
        { 0, 0, 0 },
        { 0, 0, 0 },
        { 0, 0, 0 },
};

uint8_t gucRBLEDColors[RB_LED_POS_MSG_MAX][RB_LED_DATA_PKT_MSG_MAX][RB_LED_COLOR_MAX] = { 
        { { 191, 0, 63 }, { 127, 0, 127 }, { 63, 0, 191 }, { 0, 0, 255 } }, /* RB_LED_POS_BOOT_UP_PROGRESS, (Red, Green, Blue) */
        { { 0, 0, 255 }, { 255, 0, 0 }, { 0, 255, 0 }, { 0, 0, 0 } }, /* RB_LED_POS_DATA_PKT */
        { { 0, 0, 255 }, { 255, 0, 0 }, { 0, 255, 0 }, { 0, 0, 0 } }, /* RB_LED_POS_MSG2, Blue, Green, Red, Off */
        { { 0, 0, 255 }, { 255, 0, 0 }, { 0, 255, 0 }, { 0, 0, 0 } }, /* RB_LED_POS_MSG3 */
};

/***************************************************
 *
 * Rainbow LED Functions
 *
 ***************************************************/
void RainbowLED_DebugLight(uint8_t ucLEDPos, uint8_t ucMsgId)
{
  uint8_t i;

  for( i = 0; i < RB_LED_COLOR_MAX; i++) {  /* RB_LED_COLOR_MAX : 3 */
    gucRBLEDCurrentColor[ucLEDPos][i] = gucRBLEDColors[ucLEDPos][ucMsgId][i];
  }

  for( i = 0; i < RB_LED_POS_MSG_MAX; i++) {  /* RB_LED_POS_MSG_MAX : 4 */
    pixels.setPixelColor(i, pixels.Color(gucRBLEDCurrentColor[i][1],gucRBLEDCurrentColor[i][0], gucRBLEDCurrentColor[i][2])); /* G R B */
  }
  pixels.show(); delay(10);
}

unsigned long int wheel(unsigned int i) {

  unsigned int k, m;
  unsigned int tr, tg, tb;
  
  i = i % 0x300; m = i & 0xFF; k = i & 0x300;
  switch(k) {
    case 0x0000:
      tr = 0xFF - m;      // Red Decrement
      tg = m;             // Green Increment
      tb = 0;             // Blue Off
      break;
        
    case 0x0100:
      tr = 0;             // Red Off
      tg = 0xFF - m;      // Green Decrement
      tb = m;             // Blue Increment
      break;
        
    case 0x0200:
      tr = m;             // Red Increment
      tg = 0;             // Green Off
      tb = 0xFF - m;      // Blue Decrement
      break;
  }
  gucColorRed = tr;
  gucColorGreen = tg;
  gucColorBlue = tb;
}

void RainbowLED_Test(void)
{
  unsigned int a, v;

  v = 0; a = 0;
 
  while(1) {      
    for (int i = 0; i < NUMPIXELS; i++) {
      a = i*GAP; 
      a %= 0x300;
      a += v;
      a %= 0x300;
      wheel(a);  
      pixels.setPixelColor(i, pixels.Color(gucColorGreen,gucColorRed,gucColorBlue)); 
    }
    pixels.show(); 
    delay(RAINBOWLED_DELAY);
    v += SPEED; 
    v %= 0x300;  
  }
}

#endif /* RAINBOW_LED */
