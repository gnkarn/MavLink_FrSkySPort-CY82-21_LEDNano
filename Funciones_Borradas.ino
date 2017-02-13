// desde led control
/*
// funciona para trasladar el valor del acelerador que es de 0-100 a una tira de leds de un Brazo, y una posicion, y una longitud de LedsNum
// ************************************************************
void ThrotleLed (int brazo , int posicion, int LedsNum, int hue, float dim  ){
  uint16_t  ThrotleLedNum = (ap_throttle * LedsNum)/100 ;
  //int ThrotleLedNum = map(ap_throttle,0,100,0,LedsNum);
  for ( int i=0 ; i< LedsNum;i++){
    if (i <= ThrotleLedNum)  { leds[brazo][(LED_Layout[posicion][i])]= CHSV(hue, 255, 255*dim);
    } 
      else { leds[brazo][LED_Layout[posicion][i]]= CHSV(0, 255, 0);
      }
  }
}

// ************************************************************
// funciona  led Helicoptero
// ************************************************************

void HeliLed (int bpm , int hue,float _dim  ){
 uint8_t Brazo0= beatsin8(bpm , 0, _dim ,0,0);
  uint8_t Brazo1= beatsin8(bpm , 0, _dim ,0,83);
   uint8_t Brazo2= beatsin8(bpm , 0, _dim ,0,166);
   SetBrazo(0,hue,Brazo0*(Brazo0>120));
    SetBrazo(1,hue,Brazo1*(Brazo1>120));
     SetBrazo(2,hue,Brazo2*(Brazo2>120));
}


// ************************************************************
// funciona  led Helicoptero
// ************************************************************

void SetBrazo (int brazo, int hue, int v ){
      fill_solid( leds[brazo], NUM_LEDS_PER_STRIP, CHSV(hue,255,v));
     
    }

    void rainbow()
{
  // FastLED's built-in rainbow generator
  fill_rainbow(Vir_led, NUM_LEDS, gHue, 7);
  vled_draw(); // draw from virtual to fisical leds
}

void addGlitter(fract8 chanceOfGlitter)
{
  if (random8() < chanceOfGlitter) {
    Vir_led[random16(NUM_LEDS)] += CRGB::White;
  }
  vled_draw(); // draw from virtual to fisical leds
}

void rainbowWithGlitter()
{
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80);
  vled_draw(); // draw from virtual to fisical leds
}

void juggle() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy(Vir_led, NUM_LEDS, 20);
  byte dothue = 0;
  for (int i = 0; i < 8; i++) {
    Vir_led[beatsin16(i + 7, 0, NUM_LEDS)] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
  vled_draw(); // draw from virtual to fisical leds
}

void circling()
{
  // circulo completo
  
  fadeToBlackBy(Vir_led, NUM_LEDS, 10);
  int pos = scale8(beat8(g_bpm),NUM_LEDS);
  
  Vir_led[pos] += CHSV(gHue, 255, 192);
  vled_draw(); // draw from virtual to fisical leds
}


void brazoSenoidal()
{
  // a colored dot sweeping back and forth, with fading trails en cada brazo
  
  fadeToBlackBy(leds[0], NUM_LEDS_PER_STRIP, 30);
  int pos = beatsin16(20, 0, 16);
  setArms(gHue, 250, 192, pos );
  
}

void setBandera() { //        setea cada brazo con los colores de la bandera
 
  // a colored dot sweeping back and forth, with fading trails en cada brazo
  
  fadeToBlackBy(leds[0], NUM_LEDS_PER_STRIP, 4);
  int pos = beatsin16(16, 0, 15);
  setArms(143, 175, 255, pos );  // setea todas las tiras celestes
  leds[0][LED_Layout[2][pos]] = CHSV (0,0,255); // setea la tira blanca
  memcpy(&leds[1], &leds[0][0], NUM_LEDS_PER_STRIP * 3);
  memcpy(&leds[2], &leds[0][0], NUM_LEDS_PER_STRIP * 3);
  
}


//****************************************************************
// en los brazos 1 y 2 establece una linea blanca proporcional al angulo de Roll 
// *******************************************************************

void longRoll(){
  uint8_t alfa= ((ap_roll_angle)) ; // *255/360
  fract8 longitud = sin8(alfa);
  
  uint8_t longi = map( longitud,0,255,0 ,(NUM_LEDS_PER_ARM/2));
  //setSegment (uint8_t linit,uint8_t lend, uint8_t br ,uint8_t pos, CRGB color)
  setSegment(0,longi,2,0,CRGB::White);
  setSegment(0,longi,2,1,CRGB::White);
  leds[2][LED_Layout[0][longi]]=CRGB::Red;
  leds[2][LED_Layout[1][longi]]=CRGB::Red;

  setSegment(0,NUM_LEDS_PER_ARM/2-longi,1,0,CRGB::White);
  setSegment(0,NUM_LEDS_PER_ARM/2-longi,1,1,CRGB::White);
  leds[1][LED_Layout[0][NUM_LEDS_PER_ARM/2-longi]]=CRGB::Red;
  leds[1][LED_Layout[1][NUM_LEDS_PER_ARM/2-longi]]=CRGB::Red;
  
   //Serial.print("roll : ");
    //Serial.print(ap_roll_angle);
    //Serial.print("longitud : ");
    //Serial.print(longitud);
    //Serial.print("longi : ");
    //Serial.print(longi);
  
  
  fadeToBlackBy(leds[1], NUM_LEDS_PER_STRIP, 20);
  fadeToBlackBy(leds[2], NUM_LEDS_PER_STRIP, 20);
  
}

void confetti()
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy(Vir_led, NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  Vir_led[pos] += CHSV(gHue + random8(64), 200, 255);
  vled_draw(); // draw from virtual to fisical leds
}

//#####################################################################################################
//### LARSON SCANNER (CIRCLING ON ALL LEDs)                                                         ###
//#####################################################################################################
void LarsonScanner(byte LS_hue, int START_POS, int END_POS, float dim) {
  int DELAY = 50;
  if (pos == 0) {
    pos = START_POS;
  }
  else if (pos < START_POS) {
    pos = START_POS;
  }
  else if (pos > END_POS) {
    pos = START_POS;
  }

  if (currentmillis >= targetmillis_LS) {
    switch (dir) {
    case RIGHT:
      pos++;
      if (pos == END_POS - 1)
        dir = LEFT;
      break;
    case LEFT:
      pos--;
      if (pos == START_POS)
        dir = RIGHT;
      break;
    }
    for (int i = 0; i < NUM_ARMS; i++) {
      for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
        leds[i][j] = CHSV(0, 0, 0);
      }
    }
      if (dir == RIGHT) {
      if (pos <= END_POS - 1 && pos >= START_POS)
                                for (int i = 0; i < NUM_ARMS; i++) {
                                  leds[i][pos] = CHSV(LS_hue, 255, 255 * dim);
                                }
      if (pos <= END_POS + 1 && pos >= START_POS + 1)
                                for (int i = 0; i < NUM_ARMS; i++) {
                                  leds[i][pos - 1] = CHSV(LS_hue, 255, 180 * dim);
                                }
      if (pos <= END_POS + 2 && pos >= START_POS + 2)
                                for (int i = 0; i < NUM_ARMS; i++) {
                                  leds[i][pos - 2] = CHSV(LS_hue, 255, 100 * dim);
                                }
      if (pos == -2)
                                for (int i = 0; i < NUM_ARMS; i++) {
                                  leds[i][pos + 2] = CHSV(LS_hue, 255, 100 * dim);
                                }
    }
    if (dir == LEFT) {
      if (pos <= END_POS - 1 && pos >= START_POS)
                                for (int i = 0; i < NUM_ARMS; i++) {
                                  leds[i][pos] = CHSV(LS_hue, 255, 255 * dim);
                                }
      if (pos <= END_POS - 2 && pos >= START_POS - 1)
                                for (int i = 0; i < NUM_ARMS; i++) {
                                  leds[i][pos + 1] = CHSV(LS_hue, 255, 180 * dim);
                                }
      if (pos <= END_POS - 3 && pos >= START_POS - 2)
                                for (int i = 0; i < NUM_ARMS; i++) {
                                  leds[i][pos + 2] = CHSV(LS_hue, 255, 100 * dim);
                                }
      if (pos == END_POS + 1)
                                for (int i = 0; i < NUM_ARMS; i++) {
                                  leds[i][pos - 2] = CHSV(LS_hue, 255, 100 * dim);
                                }
    }
    targetmillis_LS = lastmillis + DELAY;
  }
}


    */ //fin de borrado
    
