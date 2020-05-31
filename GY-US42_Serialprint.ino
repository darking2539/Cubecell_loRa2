/*
PRO MINI-----------GYUS42
VCC ---------- 5V
GND  --------- GND
A5 -------- RC (SCL)
A4 -------- TD (SDA)
*/
#include "Wire.h"
//The Arduino Wire library uses the 7-bit version of the address, so the code example uses 0x70 instead of the 8-bit 0xE0
#define SensorAddress byte(0x70)
//The sensors ranging command has a value of 0x51
#define RangeCommand byte(0x51)
//These are the two commands that need to be sent in sequence to change the sensor address
word range;
void setup() {
 Serial.begin(9600); //Open serial connection at 9600 baud
 pinMode(Vext, OUTPUT); //Open Vext
 digitalWrite(Vext, LOW);  //Open Vext 
 Wire.begin(); 

}
void loop(){
 digitalWrite(Vext, LOW);  //Open Vext 
 Wire.begin();
 takeRangeReading(); //Tell the sensor to perform a ranging cycle
 delay(100); //Wait for sensor to finish
 word range = requestRange(); //Get the range from the sensor
 Serial.print("Range: "); Serial.println(range); //Print to the user 
 digitalWrite(Vext, HIGH);  //Open Vext 
 
}

//Commands the sensor to take a range reading
void takeRangeReading(){
  Wire.beginTransmission(SensorAddress); //Start addressing
  Wire.write(RangeCommand); //send range command
  Wire.endTransmission(); //Stop and do something else now
}
//Returns the last range that the sensor determined in its last ranging cycle in centimeters. Returns 0 if there is no communication.
word requestRange(){
  Wire.requestFrom(SensorAddress, byte(2));
  if(Wire.available() >= 2){ //Sensor responded with the two bytes
  byte HighByte = Wire.read(); //Read the high byte back
  byte LowByte = Wire.read(); //Read the low byte back
  word range = (HighByte*256)+LowByte ; //Make a 16-bit word out of the two bytes for the range
  return range;
}
else {
  return word(0); //Else nothing was received, return 0
}
}
/* Commands a sensor at oldAddress to change its address to newAddress
oldAddress must be the 7-bit form of the address that is used by Wire
7BitHuh determines whether newAddress is given as the new 7 bit version or the 8 bit version of the address
If true, if is the 7 bit version, if false, it is the 8 bit version
*/
