    /*
 * MavLink_FrSkySPort
 * https://github.com/Clooney82/MavLink_FrSkySPort
 *
 * Copyright (C) 2014 Rolf Blomgren
 *  http://diydrones.com/forum/topics/amp-to-frsky-x8r-sport-converter
 *  Inspired by https://code.google.com/p/telemetry-convert/
 *    (C) 2013 Jochen Tuchbreiter under (GPL3)
 *
 *  Improved by:
 *    (2014) Christian Swahn
 *    https://github.com/chsw/MavLink_FrSkySPort
 *
 *    (2014) Luis Vale
 *    https://github.com/lvale/MavLink_FrSkySPort
 *
 *    (2015) Michael Wolkstein
 *    https://github.com/wolkstein/MavLink_FrSkySPort
 *
 *    (2015) Jochen Kielkopf
 *    https://github.com/Clooney82/MavLink_FrSkySPort

 (2015) modified by gnk adding front led flash , and many leds effects
 https://github.com/gnkarn/MavLink_FrSkySPort.git
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY, without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses>.
 *
 * Additional permission under GNU GPL version 3 section 7
 *
 * If you modify this Program, or any covered work, by linking or
 * combining it with FrSkySportTelemetry library (or a modified
 * version of that library), containing parts covered by the terms
 * of FrSkySportTelemetry library, the licensors of this Program
 * grant you additional permission to convey the resulting work.
 * {Corresponding Source for a non-source form of such a combination
 * shall include the source code for the parts of FrSkySportTelemetry
 * library used as well as that of the covered work.}
 *
 */
/*
 * ====================================================================================================
 *
 * Mavlink to FrSky X8R SPort Interface using Teensy 3.1
 *     http://www.pjrc.com/teensy/index.html
 *  based on ideas found here http://code.google.com/p/telemetry-convert/
 * ========================================================================
 *
 * Cut board on the backside to separate Vin from VUSB
 *
 * Connection on Teensy 3.1:
 * -----------------------------------
 *  SPort S --> TX1
 *  SPort + --> Vin
 *  SPort - --> GND
 *  APM Telemetry DF13-5 Pin 2 --> RX2
 *  APM Telemetry DF13-5 Pin 3 --> TX2
 *  APM Telemetry DF13-5 Pin 5 --> GND
 *
 * Note that when used with other telemetry device (3DR Radio 433 or 3DR Bluetooth tested) in parallel
 * on the same port the Teensy should only Receive, so please remove it's TX output (RX input on PixHawk or APM)
 *
 * Analog input  --> A0 (pin14) on Teensy 3.1 ( max 3.3 V ) - Not used
 *
 * This is the data we send to FrSky, you can change this to have your own set of data
 * ----------------------------------------------------------------------------------------------------
 * Data transmitted to FrSky Taranis:
 * Cell            ( Voltage of Cell=Cells/(Number of cells). [V])
 * Cells           ( Voltage from LiPo [V] )
 * A2              ( HDOP value * 25 - 8 bit resolution)
 * A3              ( Roll angle from -Pi to +Pi radians, converted to a value between 0 and 1024)
 * A4              ( Pitch angle from -Pi/2 to +Pi/2 radians, converted to a value between 0 and 1024)
 * Alt             ( Altitude from baro  [m] )
 * GAlt            ( Altitude from GPS   [m] )
 * hdg             ( Compass heading  [deg] )
 * Rpm             ( Throttle when ARMED [%] *100 + % battery remaining as reported by Mavlink)
 * VSpd            ( Vertical speed [m/s] )
 * Speed           ( Ground speed from GPS,  [km/h] )
 * T1              ( GPS status = ap_sat_visible*10) + ap_fixtype )
 * T2              ( Armed Status and Mavlink Messages :- 16 bit value: bit 1: armed - bit 2-5: severity +1 (0 means no message - bit 6-15: number representing a specific text)
 * Vfas            ( same as Cells )
 * Longitud        ( Longitud )
 * Latitud         ( Latitud )
 * Dist            ( Will be calculated by FrSky Taranis as the distance from first received lat/long = Home Position )
 * Fuel            ( Current Flight Mode reported by Mavlink )
 * AccX            ( X Axis average vibration m/s?)
 * AccY            ( Y Axis average vibration m/s?)
 * AccZ            ( Z Axis average vibration m/s?)
 * ====================================================================================================
 

 * ====================================================================================================
 *
 * Mavlink Adapted for arduino NANO
 * * Connection on nano GNK
 * -----------------------------------
 *  pin 30 APM telemetry  --> RX0
 *  pin 13 --> MAvlink led interno
 *  pin2  ---> gbled , green arduino front
 *  Pin 9  --> Frontled
 *  Pin 10 --> BAckled
 *  PIN 5  --> 2812 led  Data
 *  PIN 8 -->  Al Rx HB-02 (vde) for debug using arduino monitor on mac@    
 *  
 * ========================================================================
*/
 
/*
 * *******************************************************
 * *** Basic Includes:                                 ***
 * *******************************************************
 */

#include "GCS_MAVLink.h"
//#include "mavlink.h"
//#include "LSCM.h"
#include <FastLED.h>
#include <avr/pgmspace.h>  // para poder usar progmem


/// GPS status codes
enum GPS_Status {
	NO_GPS = 0,             ///< No GPS connected/detected
	NO_FIX = 1,             ///< Receiving valid GPS messages but no lock
	GPS_OK_FIX_2D = 2,      ///< Receiving valid messages and 2D lock
	GPS_OK_FIX_3D = 3,      ///< Receiving valid messages and 3D lock
	GPS_OK_FIX_3D_DGPS = 4, ///< Receiving valid messages and 3D lock with differential improvements
	GPS_OK_FIX_3D_RTK = 5,  ///< Receiving valid messages and 3D lock, with relative-positioning improvements
};

/*
 * *******************************************************
 * *** Basic Configuration:                            ***
 * *******************************************************
 */
#define debugSerial         btSerial
//#define debugSerialBaud     57600
#define _MavLinkSerial      Serial
#define _MavLinkSerialBaud  57600
#define START               1
#define MSG_RATE            10      // Hertz
#define AP_SYSID            1       // autopilot system id
#define AP_CMPID            1       // autopilot component id
#define MY_SYSID            123     // teensy system id
#define MY_CMPID            123     // teensy component id
#define GB_SYSID            71      // gimbal system id
#define GB_CMPID            67      // gimbal component id

#define AC_VERSION          "3.2"
//#define AC_VERSION          "3.3"


// serial comm para Bluetooth
#include <SoftwareSerial.h>
SoftwareSerial btSerial(11,8); // RX, TX
//#define DPN Serial.println //imprime en el puerto del usb, es tambien el de telemetria
#define DPL btSerial.println // imprime en el puerto para monitorear 
    
 // initialize the serial communication:

 
/*
 * *******************************************************
 * *** Enable Addons:                                  ***
 * *******************************************************
 */
//#define USE_FAS_SENSOR_INSTEAD_OF_APM_DATA              // Enable  if you use a FrSky FAS   Sensor.
//#define USE_FLVSS_FAKE_SENSOR_DATA                      // Enable  if you want send fake cell info calculated from VFAS, please set MAXCELLs according your Number of LiPo Cells
//#define USE_SINGLE_CELL_MONITOR                         // Disable if you use a FrSky FLVSS Sensor. - Setup in LSCM Tab
//#define USE_AP_VOLTAGE_BATTERY_FROM_SINGLE_CELL_MONITOR // Use this only with enabled USE_SINGLE_CELL_MONITOR
#define USE_RC_CHANNELS                                 // Use of RC_CHANNELS Informations ( RAW Input Valus of FC ) - enable if you use TEENSY_LED_SUPPORT.
#define USE_TEENSY_LED_SUPPORT                          // Enable LED-Controller functionality

/*
 * *******************************************************
 * *** Debug Options:                                  ***
 * *******************************************************
 */
// *** DEBUG MAVLink Messages:
//#define DEBUG_APM_MAVLINK_MSGS              // *Show all messages received from APM
//#define DEBUG_APM_CONNECTION                
//#define DEBUG_APM_CONNECTION1                 // *lo uso en lugar del anterior para reducir uso de memoria
//#define DEBUG_APM_HEARTBEAT                 // * MSG #0
//#define DEBUG_APM_SYS_STATUS                // *MSG #1   - not used -> use: DEBUG_APM_BAT
//#define DEBUG_APM_BAT                       // Debug Voltage and Current received from APM
//#define DEBUG_APM_GPS_RAW                   // *MSG #24
//#define DEBUG_APM_RAW_IMU                   // MSG #27  - not used -> use: DEBUG_APM_ACC
//#define DEBUG_APM_ACC                       // Debug Accelerometer
//#define DEBUG_APM_ATTITUDE                  // *MSG #30
//#define DEBUG_APM_GLOBAL_POSITION_INT_COV   // MSG #63  - planned - currently not implemented - not supported by APM
//#define DEBUG_APM_RC_CHANNELS               // MSG #65
//#define DEBUG_APM_VFR_HUD                   // *MSG #74
//#define DEBUG_APM_STATUSTEXT                // MSG #254 -
//#define DEBUG_APM_PARSE_STATUSTEXT
//#define DEBUG_GIMBAL_HEARTBEAT
//#define DEBUG_OTHER_HEARTBEAT


// *** DEBUG FrSkySPort Telemetry:
//#define DEBUG_FrSkySportTelemetry
//#define DEBUG_FrSkySportTelemetry_FAS
//#define DEBUG_FrSkySportTelemetry_FLVSS  *
//#define DEBUG_FrSkySportTelemetry_GPS
//#define DEBUG_FrSkySportTelemetry_RPM
//#define DEBUG_FrSkySportTelemetry_A3A4
//#define DEBUG_FrSkySportTelemetry_VARIO
//#define DEBUG_FrSkySportTelemetry_ACC
//#define DEBUG_FrSkySportTelemetry_FLIGHTMODE

// *** DEBUG other things:
//#define DEBUG_AVERAGE_VOLTAGE
// #define DEBUG_LIPO_SINGLE_CELL_MONITOR // Use this only with enabled USE_SINGLE_CELL_MONITOR

#define hbLed       2       /* Heartbeat LED if any, default arduino board has a LED onboard tied to pin 13. uso el D2 int11 pues el led interno se usa en SPI */
#define frontled  9 // digital pin 9 para pwm (puede usarse pin 3,5,6,9,10,11)
#define	backled 10 // digital pin 10
#define GpsLed 6 // digital pin 6 GPS status
#define motorsLedLeft 4
#define motorsLedRight 3


//#####################################################################################################
//### DEFAULT VARIABLES                                                                             ###
//##################################################################################################### antes en led_control.ino
/*
#define RIGHT 0
#define LEFT  1
#define OFF   0
#define ON    1
#define AUTO  2
#define BLINK 3
#define PWM_MIN    982  // minimum pwm value of your RC-System
#define PWM_MID   1496  // middle  pwm value of your RC-System
#define PWM_MAX   2006  // maximum pwm value of your RC-System

#ifdef SIMULATION_MODE
int LED_MODE_SWITCH;
int LED_DIMM_CHAN;
#else
#define LED_MODE_SWITCH 15   // Channel Number of RC Channel used for manual Lightmode
#define LED_DIMM_CHAN   16   // Channel Number of RC Channel used for dimming LEDs
#define LED_BPM_CHAN   14   // Channel Number of RC Channel used for dimming LEDs
#endif
*/


//#####################################################################################################
//### FASTLED CONFIG                                                                                ###
//##################################################################################################### antes en led_control.ino

#define RGBORDER  GRB // BRG para 2811, GRB para 2812
#define LEDTYPE   WS2812B

#define NUM_ARMS 4
#define NUM_LEDS_PER_STRIP 10  // 
#define BRIGHTNESS          96
#define FRAMES_PER_SECOND   30
#define NUM_LEDS_PER_ARM    10
#define NUM_LEDS NUM_LEDS_PER_ARM*NUM_ARMS      // Number of LED's
//CRGB Vir_led[NUM_LEDS]     ;                      // simula una tira continua con todos los leds de cada brazo conectados en serie
#define BRIGHTNESS          96
#define DATA_PIN    5// pin D5

//CRGB leds[NUM_ARMS][NUM_LEDS_PER_STRIP]; // 
CRGB leds[NUM_LEDS]; //

           // CRGBArray<NUM_LEDS_PER_STRIP> leds[NUM_ARMS] ; // 

unsigned long targetmillis_LEFT;
unsigned long targetmillis_RIGHT;
unsigned long targetmillis_ARMED;

int state_LEFT = 0;
int state_RIGHT = 0;
int state_ARMED = 0;

float dim = 100; // variable global para dim
int gbpm = 60; // variable global para efectos de led que quiero variar desde el SD del remoto



 
/*
 * *******************************************************
 * *** Variables Definitions:                          ***
 * *******************************************************
 */
// configure number maximum connected analog inputs(cells)
// if you build an six cell network then MAXCELLS is 6
//#define MAXCELLS 5

/*
 * *******************************************************
 * *** Mavlink Definitions:                            ***
 * *******************************************************
 */
/*
 * *******************************************************
 * *** Message #0  HEARTHBEAT                          ***
 * *******************************************************
 */
//uint8_t     ap_type               =  0; //original comentado
//uint8_t     ap_autopilot          =  0;//original comentado
uint8_t     ap_base_mode          =  0;
int32_t     ap_custom_mode        = -1;
//uint8_t     ap_system_status      =  0;//original comentado
uint8_t     ap_mavlink_version    =  0;
uint8_t ap_system_status = MAV_STATE_POWEROFF;// see MAV_STATE enum usado en Heartbeat message
#define minimum_battery_volts  (3400*4)


/*
 * *******************************************************
 * *** Message #1  SYS_STATUS                          ***
 * *******************************************************
 */
uint16_t    ap_voltage_battery    = 0;    // 1000 = 1V | Battery voltage, in millivolts (1 = 1 millivolt)
int16_t     ap_current_battery    = 0;    //   10 = 1A | Battery current, in 10*milliamperes (1 = 10 milliampere),
                                          //              -1: autopilot does not measure the current.
int8_t      ap_battery_remaining  = 0;    // Remaining battery energy: (0%: 0, 100%: 100),
                                          //              -1: autopilot estimate the remaining battery

/*
 * *******************************************************
 * *** Message #24  GPS_RAW_INT                        ***
 * *******************************************************
 */
uint8_t     ap_fixtype            =   0;    // 0 = No GPS, 1 = No Fix, 2 = 2D Fix, 3 = 3D Fix, 4 = DGPS, 5 = RTK.
                                            // Some applications will not use the value of this field unless it is at least two,
                                            // so always correctly fill in the fix.
uint8_t     ap_sat_visible        =   0;    // Number of visible Satelites

/*
 * *******************************************************
 * *** Message #30  ATTITUDE                           ***
 * *** needed to use current Angles and axis speeds    ***
 * *******************************************************
 */
//int32_t     ap_roll_angle         = 0;      // Roll angle (deg -180/180)
//int32_t     ap_pitch_angle        = 0;      // Pitch angle (deg -180/180)
//uint32_t    ap_yaw_angle          = 0;      // Yaw angle (rad)
//uint32_t    ap_roll_speed         = 0;      // Roll angular speed (rad/s)
//uint32_t    ap_pitch_speed        = 0;      // Pitch angular speed (rad/s)
//uint32_t    ap_yaw_speed          = 0;      // Yaw angular speed (rad/s)

/*
 * *******************************************************
 * *** Message #63  GLOBAL_POSITION_INT_COV            ***
 * *** Needed for Date/Time submission to RC           ***
 * *******************************************************
 */
//time_t      ap_gps_time_unix_utc  = 0;      // Timestamp (microseconds since UNIX epoch) in UTC.
                                            // 0 for unknown.
                                            // Commonly filled by the precision time source of a GPS receiver.

// Message #65 RC_CHANNELS
#ifdef USE_RC_CHANNELS
uint8_t     ap_chancount          = 0;      // Total number of RC channels being received.
                                            // This can be larger than 18, indicating that more channels are available but
                                            // not given in this message. This value should be 0 when no RC channels are available.
uint16_t    ap_chan_raw[18]       = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  // RC channel x input value, in microseconds.
                                                                            // A value of UINT16_MAX (65535U) implies the channel is unused.
#endif

/*
 * *******************************************************
 * *** Message #74  VFR_HUD                            ***
 * *******************************************************
 */

uint16_t    ap_throttle           = 0;    // Current throttle setting in integer percent, 0 to 100
//int32_t     ap_airspeed           = 0;    // Current airspeed in m/s
//uint32_t    ap_groundspeed        = 0;    // Current ground speed in m/s
uint32_t    ap_heading            = 0;    // Current heading in degrees, in compass units (0..360, 0=north)
// FrSky Taranis uses the first recieved value after 'PowerOn' or  'Telemetry Reset'  as zero altitude
//int32_t     ap_bar_altitude       = 0;    // 100 = 1m
//int32_t     ap_climb_rate         = 0;    // 100 = 1m/s
/*
 * *******************************************************
 * *** Message #253  MAVLINK_MSG_ID_STATUSTEXT         ***
 * *******************************************************
 */
uint16_t    ap_status_severity    = 255;
uint16_t    ap_status_send_count  =   0;
uint16_t    ap_status_text_id     =   0;
mavlink_statustext_t statustext;
/*
  MAV_SEVERITY_EMERGENCY=0,     System is unusable. This is a "panic" condition.
  MAV_SEVERITY_ALERT=1,         Action should be taken immediately. Indicates error in non-critical systems.
  MAV_SEVERITY_CRITICAL=2,      Action must be taken immediately. Indicates failure in a primary system.
  MAV_SEVERITY_ERROR=3,         Indicates an error in secondary/redundant systems.
  MAV_SEVERITY_WARNING=4,       Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning.
  MAV_SEVERITY_NOTICE=5,        An unusual event has occured, though not an error condition. This should be investigated for the root cause.
  MAV_SEVERITY_INFO=6,          Normal operational messages. Useful for logging. No action is required for these messages.
  MAV_SEVERITY_DEBUG=7,         Useful non-operational messages that can assist in debugging. These should not occur during normal operation.
  MAV_SEVERITY_ENUM_END=8,
*/


/*
 * *******************************************************
 * *** These are special for FrSky:                    ***
 * *******************************************************
 */
int32_t     gps_status            = 0;    // (ap_sat_visible * 10) + ap_fixtype
                                          // ex. 83 = 8 sattelites visible, 3D lock
//uint8_t     ap_cell_count         = 0;

/*
 * *******************************************************
 * *** Variables needed for Mavlink Connection Status  ***
 * *** and starting FrSkySPort Telemetry               ***
 * *******************************************************
 */
bool          MavLink_Connected     =     0;  // Connected or Not
//unsigned long start_telemetry       = 30000;  // Start Telemetry after 30s (or 5s after init)
bool          telemetry_initialized =     0;  // Is FrSkySPort Telemetry initialized


/*
 * *******************************************************
 * *** End of Variables definition                     ***
 * *******************************************************
 */

/*
 * *******************************************************
 * *** Setup:                                          ***
 * *******************************************************
 */
void setup()  {


  // bluetooth serial
  btSerial.begin(38400);// ahora lo uso para monitor
  DPL(F("Mav2led monitor"));  

  
   pinMode (hbLed, OUTPUT); // salida de hbled uso el A3 Pcint 11 pues el 13 es SCK
  #ifdef USE_TEENSY_LED_SUPPORT
    Teensy_LED_Init();        // Init LED Controller
  #endif

  Mavlink_setup();           // Init Mavlink
  LedInit(frontled);         // front led gnk init
  LedInit(backled);
  LedInit(motorsLedLeft);		// motors left Led
  LedInit(motorsLedRight);		// motors right leds
  LedInit(GpsLed);		// motors right leds

 
          
}

/*
 * *******************************************************
 * *** Main Loop:                                      ***
 * *******************************************************
 */
void loop()  {

 

  _MavLink_receive();                       // receive MavLink communication
  

  Mavlink_check_connection();               // Check MavLink communication

  if ( MavLink_Connected == 1 ) {           // If we have a valid MavLink Connection
	  //if ((mavlink_active)) digitalWrite(hbLed, ioCounter == 1 ? HIGH : LOW); 
    leds[1]=(((millis() % 250) >= 200) ? CRGB ::White : 0); // El led uno titila si mavlink_activo
    
      Teensy_LED_process();                 // Process LED-Controller
  
  }
  else
  {
	  default_mode(0, 150); // if mavlink not connected put leds in a default state
    LEDS.show();
     
  }
  
  FrontFlash(64, 255);  //  hace titilar el led flash de frente GNK cambiar 255 por dim
 // EVERY_N_MILLISECONDS(500) {digitalWrite(hbLed,!digitalRead(hbLed));  } //Blink(boolean LedActivo,byte LedPin , uint8_t Ton)
  
  digitalWrite( hbLed ,((millis() % 500) >= 100)) ; // una formA DE  generar onda rectangula de cualquier % ciclo - en este caso aplicado al led sobre el arduino
  //GenericFlash(uint8_t pin, uint8_t pw, uint8_t offset, int BPM, float dim) 
  GenericFlash(backled, 32, 160, 60, 256); // flash del led  trasero
  //GenericFlash(13, 50, 160, 60, 200); // flash de mavlink para pruebas , normalmente debe parpadear cuando se establece la coneccion
 
  
}
