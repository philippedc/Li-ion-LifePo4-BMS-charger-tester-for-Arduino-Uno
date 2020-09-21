/*
BMS Li-ion/LifoPo4 automatic charger 1S 2S 3S 4S
  _________________________________________________________________
  |                                                               |
  |       author : Philippe de Craene <dcphilippe@yahoo.fr        |
  |       Free of use - Any feedback is welcome                   |
  _________________________________________________________________

The charger works in 2 parts:
- First the BMS part from the idea of https://simple-ee.com/2019/07/20/arduino-4s-bms-version-7/
- Then the charger part with a buck converter

Calibration of measured voltages are done with the console. Connect the Arduino Uno to the usb of the PC.

The model (Li-ion or LifePo4) of cell and the number (1 to 4) to be charged are set with the binary rotary button.

When the yellow LED assigned to each cell is continually lighted, the cell is charged (the shunt is active).


Arduino Uno pinout
------------------

  A0  ==> B1 input
  A1  ==> B2 input
  A2  ==> B3 input
  A3  ==> B4 input
  A4  ==> SSD1306 display SDA
  A5  ==> SSD1306 display SCL
   2  ==> B1 shunt output - pin2 of 1A / SN754410
   3  ==> B2 shunt output - pin7 of 2A / SN754410
   4  ==> B3 shunt output - pin10 of 3A / SN754410
   5  ==> B4 shunt output - pin15 of 4A / SN754410
   6  ==> input from Imax charging sensor
   7  ==> buzzer
   9  ==> PWM output for buck converter
  10  ==> x1 BCD rotary contactor
  11  ==> x10 BCD rotary contactor
  12  ==> x100 BCD rotary contactor
  13  ==> ENABLE output for SN754410

Versions history
----------------
  version 0.1 - 21 august 2020 - first operational version
  version 0.3 - 1 sept 2020    - add the limit charging current 
  version 1.0 - 16 sept 2020   - add the buck converter

Remarks
-------

About Serial.print(F("bla bla") usage see https://www.baldengineer.com/arduino-f-macro.html
RAM usage decrease from 81% to 34% inside this code  
*/

#include <EEPROM.h>
#include "ssd1306.h"      // https://github.com/lexus2k/ssd1306

// Parameters
//-----------

const bool FIRST_USE = false;  // must be set "true" the very first use to record parameters in EEPROM
bool cellModel = LOW;          // LOW = Li-ion / HIGH = LifePo4
byte cellNumber = 4;           // number of cells to charge
float Vmax[2] = { 4.2, 3.7 };  // maximum voltage for Li-ion / LifePo4 cells
float Vmin[2] = { 3.6, 3.2 };  // minimum volatge for Li-ion / LifePo4 cells
float Vcalibration[4] = { 1.0, 1.0, 1.0, 1.0 };  // calibration to fit real voltage measures

// Hardware connexion
//-------------------

byte VcellPin[4] = { 0, 1, 2, 3 };  // analog read of each cell : A0:B1, A1:B2, A2:B3, A3:B4
byte shuntPin[4] = { 2, 3, 4, 5 };  // output to shunt for each cell
const byte ImaxPin   = 6;      // digital input from Imax charging sensor/detector
const byte buzzerPin = 7;      // alarm undervoltage cell
const byte pwmPin    = 9;      // pwm for buck converter
const byte UrcPin    = 10;     // x1 BCD rotary contactor
const byte DrcPin    = 11;     // x10 BCD rotary contactor
const byte CrcPin    = 12;     // x100 BCD rotary contactor
const byte enablePin = 13;     // SN754410 ENABLE

// Global variables
//-----------------

float Valim = 0;               // power supply voltage
float Vcell[4] = { 4.9, 4.9, 4.9, 4.9 };       // cells measured voltage
float memo_Vcell[4] = { 0.0, 0.0, 0.0, 0.0 };  // past cells voltage
bool runOnce = true;           // run one time only flag
byte pwm = 0;                  // buck converter pwm
bool parametersMenu = false;   // flag when parameters are set by the console inputs
byte index = 0;                // input character counter
char consoleInput[7];          // console input
char whatToDisplay = 'A';      // select console messages to display, 'A' for all
bool Imax = false;             // flag for charging max current 
bool memo_Urc, memo_Drc, memo_Crc;

//
// setup
//____________________________________________________________________________________________

void setup() {

// define inputs & outputs
  pinMode( enablePin, OUTPUT ); digitalWrite( enablePin, LOW );
  for( byte i=0; i<4; i++ ) {
    pinMode( shuntPin[i], OUTPUT ); digitalWrite( shuntPin[i], LOW );
  }
  pinMode( pwmPin, OUTPUT ); analogWrite( pwmPin, 0 );
  pinMode( UrcPin, INPUT_PULLUP );
  pinMode( DrcPin, INPUT_PULLUP );
  pinMode( CrcPin, INPUT_PULLUP );
  pinMode( ImaxPin, INPUT_PULLUP );
  pinMode( buzzerPin, OUTPUT );

// Initialise the oled display & console
  Serial.begin(250000);
  Serial.println(F("Starting...."));

  ssd1306_128x32_i2c_init();
  //ssd1306_128x64_i2c_init();
  ssd1306_fillScreen(0x00);
  ssd1306_setFixedFont(ssd1306xled_font6x8);
  ssd1306_clearScreen();

// EEPROM check and data upload :
// stored data are always positive from 0 to 255.
// it seems that in cas of first use all are set to 255.
  if( FIRST_USE ) EEPROM_Update();
  else EEPROM_Get();

// set Timer1 pin9 and pin10 
  TCCR1B = TCCR1B & B11111000 | B00000001;  // set Timer1 divisor to 1 for PWM frequency of 31372.55 Hz

// last tasks....
  RotactorConfig();                 // check configuration from binary rotary contactor
  digitalWrite( enablePin, HIGH );  // enable SN754410

}      // end of setup

//
// loop
//____________________________________________________________________________________________

void loop() {

  static float VcellCumul[4] = { 0.0, 0.0, 0.0, 0.0 };         // cumulative analogRead in bytes
  static unsigned int analogReadsCount = 0;                    // number of analogRead counter
  static bool buzzerON = false;
  static bool oneMinute = false;

  unsigned long tempo = millis();

// this is run only one time or after any config change or every 5 minutes
//------------------------------------------------------------------------
  if( runOnce ) {
    static byte counter = 0;
    static unsigned long memo_tempo_shuntON = 0;
    buzzerON = false;                                          // stop the buzzer if any
    for( byte i=cellNumber; i<4; i++ ) digitalWrite( shuntPin[i], HIGH );   // shunt inactive cells
    if( counter > 5 ) {
      counter = 0;
      runOnce = false;
    }
    else if( tempo - memo_tempo_shuntON > 300 ) {
      memo_tempo_shuntON = tempo;
      counter++;
      for( byte i=0; i<cellNumber; i++ ) digitalWrite( shuntPin[i], HIGH ); delay(1);
      for( byte i=0; i<cellNumber; i++ ) digitalWrite( shuntPin[i], LOW );  // active cells reset shunt
    }
  }    // end of test runOnce

// this is run every loop cycle
//-----------------------------
// read the analog values
  for( byte i=0; i<4; i++ ) {
    int VcellByte = analogRead( VcellPin[i] );
    delay(1);
    VcellCumul[i] += VcellByte;
  }
  Imax = !digitalRead( ImaxPin );                 // Imax reached when IMax 'true'

// define the DC-DC buck converter pwm ratio
// choose and comment the test with or without power voltage safety
  //if( Imax || ( Valim >= (4*Vmax[cellModel])+0.5)) {    // test with power voltage safety
  if( Imax || ( Vcell[0] < 4.9)) {                      // take care B1 does not exceed 5V !
    if( pwm > 0 ) pwm--; 
  }
  else if( pwm < 255 ) pwm++;
 
  analogWrite( pwmPin, pwm );
 

// the following is done after every analogReadsCount cycles of reading analog values
//-----------------------------------------------------------------------------------
  if( ++analogReadsCount > 100 ) return; 

// calculate voltages
  for( byte i=0; i<4 ; i++ ) {
    Vcell[i] = (VcellCumul[i] / (float)analogReadsCount / 1023.0) * 5.0 * Vcalibration[i] * (i+1);
    VcellCumul[i] = 0;
  }
  analogReadsCount = 0;
  Valim = Vcell[3];
  for( byte i=3; i>0; i-- ) {
    Vcell[i] -= Vcell[i-1];  // get the voltage for each cell
  }    // end of for

// this is done every second: read buttons and display data
//---------------------------------------------------------
  static unsigned long memo_tempo = 0;
  if( tempo - memo_tempo < 1000 ) return;
  memo_tempo = tempo;

// check balancing and undervoltage
  for( byte i=0; i<4; i++ ) {
    if( Vcell[i] > Vmax[cellModel] ) digitalWrite( shuntPin[i], HIGH );    // check balancing
    else if( oneMinute && ( Vcell[i] < Vmin[cellModel])) buzzerON = true;  // check undervoltage
  }    // end of for
  if( buzzerON ) tone( buzzerPin, 440, 50 );           // one short tone every seconds

// check configuration from binary rotary contactor
  RotactorConfig();

// console display
  ConsoleDisplay( whatToDisplay );

// oled display
  OledDisplay();
 
// this is done every 60 seconds: voltage tendencies
//-------------------------------------------------
  static unsigned long memo_tempo_tendancy = 0;
  
  if( tempo - memo_tempo_tendancy < 60000 ) return;
  memo_tempo_tendancy = tempo;
  for( byte i=0; i<4; i++ ) memo_Vcell[i] = Vcell[i];     // remember past cells voltages for tendancies
  oneMinute = true;
  runOnce = true;
  
}      // end of loop

//============================================================================================
// list of functions
//============================================================================================

// check configuration from binary rotary contactor
//____________________________________________________________________________________________

void RotactorConfig() {

  bool Urc = digitalRead( UrcPin );
  bool Drc = digitalRead( DrcPin );
  bool Crc = digitalRead( CrcPin );
  if( Crc ) cellModel = HIGH;        // LifePo4 model cell
  else cellModel = LOW;              // Li-ion model cell
  if( Urc ) {
    if( Drc ) cellNumber = 4;
    else cellNumber = 2;
  }
  else {
    if( Drc ) cellNumber = 3;
    else cellNumber = 1;
  }
  if((Urc != memo_Urc) || (Drc != memo_Drc) || (Crc != memo_Crc)) runOnce = true;
  memo_Urc = Urc;
  memo_Drc = Drc;
  memo_Crc = Crc;
}      // end of RotactorConfig()

//
// TendancySet() : set the tendancy of cells voltage
//____________________________________________________________________________________________

char TendancySet( byte i ) {

  if( Vcell[i] - memo_Vcell[i] > 0.0 ) return '+';
  else if((Vcell[i] - memo_Vcell[i]) == 0 ) return '=';
  else return '-';
  return '?';
}      // end of TendancySet()

//
// EEPROM_Get() : read values stored in the EEPROM
//____________________________________________________________________________________________

void EEPROM_Get() {

  for( byte i=0; i<4; i++ ) {
    int var = (EEPROM.read(2*i) << 8) + EEPROM.read((2*i)+1);
    Vcalibration[i] = var/1000.0;
  }
  Vmax[0] = (EEPROM.read(8)+300)/100.0;
  Vmax[1] = (EEPROM.read(9)+300)/100.0;
  Vmin[0] = (EEPROM.read(10)+300)/100.0;
  Vmin[1] = (EEPROM.read(11)+300)/100.0;
}      // end of EEPROM_Get()

//
// EEPROM_Update() : update values stored in the EEPROM
//____________________________________________________________________________________________

void EEPROM_Update() {
  for( byte i=0; i<4; i++ ) {
    int var = 1000*Vcalibration[i];
    EEPROM.update((2*i), highByte(var));
    EEPROM.update(((2*i)+1), lowByte(var));
  }
  EEPROM.update(8, ((Vmax[0]*100.0)-300));
  EEPROM.update(9, ((Vmax[1]*100.0)-300));
  EEPROM.update(10, ((Vmin[0]*100.0)-300));
  EEPROM.update(11, ((Vmin[1]*100.0)-300));

}      // end of EEPROM_Update()

//
// OledDisplay() : display to Oled
//____________________________________________________________________________________________

void OledDisplay() {
  char flt2str[6];

  //first line
  ssd1306_printFixed ( 0,  0, "BMS", STYLE_NORMAL);
  dtostrf( cellNumber, 1, 0, flt2str );  // usage : ( number_value, number_of_digits, nulber_of_decimal, char_output)
  ssd1306_printFixed (20,  0, flt2str, STYLE_NORMAL);
  ssd1306_printFixed (30,  0, "x", STYLE_NORMAL);
  if( cellModel ) ssd1306_printFixed (40,  0, "LifePo4", STYLE_NORMAL);
  else ssd1306_printFixed (40,  0, "Li-ion", STYLE_NORMAL);

  dtostrf( Valim, 4, 1, flt2str );
  ssd1306_printFixed (96, 0, flt2str, STYLE_NORMAL);
  ssd1306_printFixed (120,  0, "V", STYLE_NORMAL);

  //cells value: B2, B1, B4, B3
  if( Vcell[1] < 0 ) dtostrf( Vcell[1], 5, 2, flt2str );
  else dtostrf( Vcell[1], 5, 3, flt2str );
  flt2str[4] = TendancySet(1);
  ssd1306_printFixed (0,  16, "B2 ", STYLE_NORMAL);
  ssd1306_printFixed (20, 16, flt2str, STYLE_NORMAL);

  if( Vcell[0] < 0 ) dtostrf( Vcell[0], 5, 2, flt2str );
  else dtostrf( Vcell[0], 5, 3, flt2str );
  flt2str[4] = TendancySet(0);
  ssd1306_printFixed (0,  24, "B1 ", STYLE_NORMAL);
  ssd1306_printFixed (20, 24, flt2str, STYLE_NORMAL);

  if( Vcell[3] < 0 ) dtostrf( Vcell[3], 5, 2, flt2str );
  else dtostrf( Vcell[3], 5, 3, flt2str );
  flt2str[4] = TendancySet(3);
  ssd1306_printFixed (64, 16, "B4 ", STYLE_NORMAL);
  ssd1306_printFixed (84, 16, flt2str, STYLE_NORMAL);

  if( Vcell[2] < 0 ) dtostrf( Vcell[2], 5, 2, flt2str );
  else dtostrf( Vcell[2], 5, 3, flt2str );
  flt2str[4] = TendancySet(2);
  ssd1306_printFixed (64, 24, "B3 ", STYLE_NORMAL);
  ssd1306_printFixed (84, 24, flt2str, STYLE_NORMAL);
}      // end of OledDisplay()

//
// ConsoleDisplay() : console displays
//____________________________________________________________________________________________

void ConsoleDisplay( char what ) {
  
    Serial.println(F("\nBMS charger/tester general menu, type the command according to desired action[:value], then ENTER\n"));
  if( what == 'B' || what == 'A' ) {
    Serial.println(F("Bxaaaa with x=1..4 for cell number x with a.aaa the new voltage calibration value"));
    Serial.print(F(" Cal1= ")); Serial.print( Vcalibration[0],3 ); Serial.print(F(" "));
    Serial.print(F(" Cal2= ")); Serial.print( Vcalibration[1],3 ); Serial.print(F(" "));
    Serial.print(F(" Cal3= ")); Serial.print( Vcalibration[2],3 ); Serial.print(F(" "));
    Serial.print(F(" Cal4= ")); Serial.print( Vcalibration[3],3 ); Serial.println();
    Serial.print(F(" B1= "));  Serial.print( Vcell[0],2 ); Serial.print(F("    "));
    Serial.print(F(" B2= "));  Serial.print( Vcell[1],2 ); Serial.print(F("    "));
    Serial.print(F(" B3= "));  Serial.print( Vcell[2],2 ); Serial.print(F("    "));
    Serial.print(F(" B4= "));  Serial.print( Vcell[3],2 ); Serial.println("\n");
  }
  if( what == 'H' || what == 'A' ) {
    Serial.println(F("HTaaa for Li-ion to set the cell maximum voltage to a.aa"));
    Serial.println(F("HFaaa for LifePo4 to set the cell maximum voltage to a.aa"));
    Serial.print(F(" Li-ion Vmax = ")); Serial.print( Vmax[0]); 
    Serial.print(F("    LifePo4 Vmax = ")); Serial.println( Vmax[1]);
    Serial.println();
  }
   if( what == 'L' || what == 'A' ) {
    Serial.println(F("LTaaa for Li-ion to set the cell minimum voltage to a.aa"));
    Serial.println(F("LFaaa for LifePo4 to set the cell minimum voltage to a.aa"));
    Serial.print(F(" Li-ion Vmin = ")); Serial.print( Vmin[0]); 
    Serial.print(F("    Lifepo4 Vmin = ")); Serial.println( Vmin[1]);
    Serial.println();
  }
    Serial.print(F(" Actual : ")); 
    if( cellModel ) Serial.print(F("LifePo4"));
    else Serial.print(F("Li-ion"));
    Serial.print(F("   pwm = ")); Serial.print(pwm); Serial.print(F("   power supply = ")); 
    Serial.print(Valim); Serial.print(F(" / ")); Serial.print(4*Vmax[cellModel]); Serial.println(F(" maxi\n"));
  
    Serial.println(F("S to save data to EEPROM"));
    Serial.println(F("E to recover last saved data\n\n\n"));

}      // end of ConsoleDisplay()

//
// serialEvent() : Arduino builtin function for any console input
//____________________________________________________________________________________________

void serialEvent() {

  if( Serial.available() ) {
    char incomingChar = Serial.read();      // no timeout nor delay unlike Serial.readBytesUntil()
    if( incomingChar != '\n' ) {
      consoleInput[index] = incomingChar;
      index++;
    }
    else {
      consoleInput[index] ='\0';            // null character
      index = 0;
      
      int val, cell;
      switch(consoleInput[0]) {
        case 'b':
        case 'B': whatToDisplay = 'B';
                  val = (consoleInput[5]-48)+(10*(consoleInput[4]-48))+(100*(consoleInput[3]-48))+(1000*(consoleInput[2]-48));
                  cell = consoleInput[1] -49;  // -48 (because ASCII) -1 (because 1..4 becomes 0..3
                  if( cell > cellNumber ) Serial.println(F("cell number out of range"));
                  else if( val < 800 || val > 1200 ) Serial.println(F("Calibration value out of range"));
                  else Vcalibration[cell] = val/1000.0; 
                  break;
        case 'l':
        case 'L': whatToDisplay = 'L';
                  val = (consoleInput[4]-48)+(10*(consoleInput[3]-48))+(100*(consoleInput[2]-48));
                  if( val < 270 || val > 460 ) {
                    Serial.println(F("cell voltage value out of range"));
                    break;
                  }
                  if((consoleInput[1] == 't') || (consoleInput[1] == 'T')) Vmin[0] = val/100.0;
                  else if((consoleInput[1] == 'f') || (consoleInput[1] == 'F')) Vmin[1] = val/100.0;
                  else Serial.println(F("unrecognised cells model"));
                  break;
        case 'h':
        case 'H': whatToDisplay = 'H';
                  val = (consoleInput[4]-48)+(10*(consoleInput[3]-48))+(100*(consoleInput[2]-48));
                  if( val < 270 || val > 460 ) {
                    Serial.println(F("cell voltage value out of range"));
                    break;
                  }
                  if((consoleInput[1] == 't') || (consoleInput[1] == 'T')) Vmax[0] = val/100.0;
                  else if((consoleInput[1] == 'f') || (consoleInput[1] == 'F')) Vmax[1] = val/100.0;
                  else Serial.println(F("unrecognised cells model"));
                  break;
        case 'e':
        case 'E': whatToDisplay = 'A';
                  Serial.println(F("cancel all changes!"));
                  EEPROM_Get();
                  break;
        case 's':
        case 'S': whatToDisplay = 'A';
                  Serial.println(F("all data saved!"));
                  EEPROM_Update();
                  break;

        default : whatToDisplay = 'A';
                  Serial.println(F("unrecognised command"));
                  break;
                  
      }  // end of switch
    }    // en of else
  }      // end of test Serial.available()
}        // end of serialEvent()
