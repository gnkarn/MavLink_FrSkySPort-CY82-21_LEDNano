// by gnk to add frontled functionallity

 //#include <FastLED.h>
 
 


//#####################################################################################################
//### gnk LED frente  INICIALIZACION                                                                               ###
//#####################################################################################################
//    lscm.setCustomCellDivider(2545.3023,1325.3131,861.8841,659.6568,527.7438); // preparado para 5 celdas  esta linea colocarla en Mavlink_Frsky-clooney82-21.ino 

  void LedInit(uint8_t ledpin){
    pinMode(ledpin,OUTPUT); // led de frente salida analogicA

    for (int i = 0; i < 200; i++) {// hace un pulso en el led de frente
    analogWrite(ledpin, 255-i);
	delay(10);
    }
    
    analogWrite(ledpin,0);
    delay(1000);
    analogWrite(ledpin,200);
    delay(1000);
    analogWrite(ledpin,0);

  }


//################################################################################
//### FRONT FLASH GNK                                                          ###
//################################################################################
void FrontFlash(int BPM, float dim) {
  // uint8_t   beat8 (accum88 beats_per_minute, uint32_t timebase=0)
 // beat8 generates an 8-bit 'sawtooth' wave at a given BPM 
    
    
    // squarewave8: square wave generator.

  /** Useful for turning a one-byte ever-increasing value into a one-byte value that is either 0 or 255. The width of the output 'pulse' is determined by the pulsewidth argument:
    uint8_t squarewave8 (uint8_t in,uint8_t pulsewidth = 128 )
    
    1 If pulsewidth is 255, output is always 255.
    2 If pulsewidth < 255, then
    3   if input < pulsewidth  then output is 255
    4   if input >= pulsewidth then output is 0
the output looking like:

    1 255   +--pulsewidth--+
    2  .    |              |
    3  0    0              +--------(256-pulsewidth)--------
    
    **/
    
    int A= squarewave8(beat8(BPM,0),32);// GENERA PULSOS APROX CADA 1 SEGUNDO DE UNA DURACION DE (32/256) * 60*1000/bpm MILISECS
    int B= squarewave8(beat8(BPM/4,200),8);
    A=255-min(254,A+B); // combina las dos ondas la de 1 por segundo y la de uno cada cuatro superponiendolas
    analogWrite(frontled,A);
      //Serial.print("a out ");
      //Serial.print(A);
    }


uint8_t beatsaw8(accum88 beats_per_minute, uint8_t lowest = 0, uint8_t highest = 255,
  uint32_t timebase = 0, uint8_t phase_offset = 0)
{
  uint8_t beat = beat8(beats_per_minute, timebase);
  uint8_t beatsaw = beat + phase_offset;
  uint8_t rangewidth = highest - lowest;
  uint8_t scaledbeat = scale8(beatsaw, rangewidth);
  uint8_t result = lowest + scaledbeat;
  return result;
}


//################################################################################
//###     GenericFlash                                                         ###
//################################################################################
void GenericFlash(uint8_t pin, uint8_t pw, uint8_t offset, int BPM, uint8_t dim) {
	// uint8_t   beat8 (accum88 beats_per_minute, uint32_t timebase=0)
	// beat8 generates an 8-bit 'sawtooth' wave at a given BPM 


	// squarewave8: square wave generator.

	/** Useful for turning a one-byte ever-increasing value into a one-byte value that is either 0 or 255. The width of the output 'pulse' is determined by the pulsewidth argument:
	uint8_t squarewave8 (uint8_t in,uint8_t pulsewidth = 128 )

	1 If pulsewidth is 255, output is always 255.
	2 If pulsewidth < 255, then
	3   if input < pulsewidth  then output is 255
	4   if input >= pulsewidth then output is 0
	the output looking like:

	1 255   +--pulsewidth--+
	2  .    |              |
	3  0    0              +--------(256-pulsewidth)--------

	**/

	int A = 255-squarewave8(beat8(BPM, offset), pw);// GENERA PULSOS APROX CADA 1 SEGUNDO DE UNA DURACION DE (32/256) * 60*1000/bpm MILISECS
	//A = scale8(A, dim);
	analogWrite(pin, A);
	//Serial.print("a out ");
	//Serial.print(A);
}

