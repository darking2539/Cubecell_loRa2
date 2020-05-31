#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "Wire.h"
#include "TinyGPS++.h"
#include <softSerial.h>

//GPS parameter
softSerial serial_connection(13, GPIO3); //
TinyGPSPlus gps;//This is the GPS object that will pretty much do all the grunt work with the NMEA data
float latidute,longitude;
int mode = 0;

//Ultrasonics GY-US42 parameter
#define SensorAddress byte(0x70) //The Arduino Wire library uses the 7-bit version of the address, so the code example uses 0x70 instead of the 8-bit 0xE0
#define RangeCommand byte(0x51) //The sensors ranging command has a value of 0x51
int range;


/* OTAA para*/
uint8_t devEui[] = { 0x30, 0x59, 0x23, 0x19, 0x02, 0x20, 0x23, 0x32 }; //3059231902202331
uint8_t appEui[] = { 0x30, 0x59, 0x23, 0x19, 0x02, 0x20, 0x23, 0x31 };
uint8_t appKey[] = { 0x16, 0x28, 0xae, 0x2b, 0x7e, 0x15, 0xd2, 0xa6, 0xab, 0xf7, 0xcf, 0x4f, 0x3c, 0x15, 0x88,0x09 };
/* ABP para*/
uint8_t nwkSKey[] = { 0x28, 0xae, 0xd2, 0x2b, 0x7e, 0x15, 0x16, 0xa6, 0x09, 0xcf, 0xab, 0xf7, 0x15, 0x88, 0x4f,0x3c };     //"28AED22B7E1516A609CFABF715884F3C"
uint8_t appSKey[] = { 0x16, 0x28, 0xae, 0x2b, 0x7e, 0x15, 0xd2, 0xa6, 0xab, 0xf7, 0xcf, 0x4f, 0x3c, 0x15, 0x88,0x09 };     //"1628AE2B7E15D2A6ABF7CF4F3C158809"
uint32_t devAddr =  ( uint32_t )0x007e6ae5;

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 30000;

/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = 2;
/*!
* Number of trials to transmit the frame, if the LoRaMAC layer did not
* receive an acknowledgment. The MAC performs a datarate adaptation,
* according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
* to the following table:
*
* Transmission nb | Data Rate
* ----------------|-----------
* 1 (first)       | DR
* 2               | DR
* 3               | max(DR-1,0)
* 4               | max(DR-1,0)
* 5               | max(DR-2,0)
* 6               | max(DR-2,0)
* 7               | max(DR-3,0)
* 8               | max(DR-3,0)
*
* Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
* the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 8;

/* Prepares the payload of the frame */
static void prepareTxFrame( uint8_t port )
{
  /*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
  *appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
  *if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
  *if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
  *for example, if use REGION_CN470, 
  *the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
  */
    digitalWrite(Vext, LOW);  //Open Vext 
    unsigned char *puc1, *puc2, *puc3, *puc4 ;

    //Ultrasonics Reader 
    delay(100); //wait Vext open
    range = 0;  //reset range

    takeRangeReading(); //Tell the sensor to perform a ranging cycle
    delay(100); //Wait for sensor to finish
    requestRange(); //Get the range from the sensor
    
    takeRangeReading(); //Tell the sensor to perform a ranging cycle
    delay(100); //Wait for sensor to finish
    requestRange(); //Get the range from the sensor
    
    int distance_read = int(range)*100; 
    puc1 = (unsigned char *)(&distance_read);

    // GPS reader
    if(mode<20) {
      mode++ ;
      Serial.println("Ignore Read GPS");
    }
    else {             
    readGPS();
    mode = 0;
    }
    int latitude_send = gps.location.lat()*10000; 
    int longitude_send = gps.location.lng()*10000;
    puc2 = (unsigned char *)(&latitude_send);
    puc3 = (unsigned char *)(&longitude_send);

    Serial.print("range!! :");
    Serial.println(range);

    Serial.print("mode:");
    Serial.println(mode);

    //battery reader
    int voltage = getBatteryVoltage()/10; //this voltage = realvoltage*10
    puc4 = (unsigned char *)(&voltage);

    
    appDataSize = 0; 
    appData[appDataSize++] = 0x00; //GPS Location
    appData[appDataSize++] = 0x88;
    appData[appDataSize++] = puc2[2];  //Latitude
    appData[appDataSize++] = puc2[1];
    appData[appDataSize++] = puc2[0];
    appData[appDataSize++] = puc3[2];  //Longtitude
    appData[appDataSize++] = puc3[1];
    appData[appDataSize++] = puc3[0];
    appData[appDataSize++] = 0x00;  //Fixed Altitude
    appData[appDataSize++] = 0x00;
    appData[appDataSize++] = 0x00;

    appData[appDataSize++] = 0x01; //ultrasonic parameter
    appData[appDataSize++] = 0x02;
    appData[appDataSize++] = puc1[1];
    appData[appDataSize++] = puc1[0];

    appData[appDataSize++] = 0x02; //Battery Monitoring
    appData[appDataSize++] = 0x02;
    appData[appDataSize++] = puc4[1];
    appData[appDataSize++] = puc4[0];

    appData[appDataSize++] = 0x03; //fixed temp parameter
    appData[appDataSize++] = 0x67;
    appData[appDataSize++] = 01;
    appData[appDataSize++] = 10;
    
    Serial.println("complete prepare");
    digitalWrite(Vext, HIGH);  //Close Vext

}


void setup() {
  boardInitMcu();
  Serial.begin(115200);
  pinMode(Vext, OUTPUT); //Open Vext

  //GPS
  serial_connection.begin(9600);//This opens up communications to the GPS
  pinMode(GPIO0, INPUT); //PPS Status(GPS)

  //Ultrasonics
  Wire.begin();

  deviceState = DEVICE_STATE_INIT;
  LoRaWAN.ifskipjoin();
}

void loop()
{
  switch( deviceState )
  {
    case DEVICE_STATE_INIT:
    {
#if(AT_SUPPORT)
      getDevParam();
#endif
      printDevParam();
      LoRaWAN.init(loraWanClass,loraWanRegion);
      deviceState = DEVICE_STATE_JOIN;
      break;
    }
    case DEVICE_STATE_JOIN:
    {
      LoRaWAN.join();
      break;
    }
    case DEVICE_STATE_SEND:
    {
      prepareTxFrame( appPort );
      LoRaWAN.send();
      deviceState = DEVICE_STATE_CYCLE;
      break;
    }
    case DEVICE_STATE_CYCLE:
    {
      // Schedule next packet transmission
      txDutyCycleTime = appTxDutyCycle + randr( 0, APP_TX_DUTYCYCLE_RND );
      LoRaWAN.cycle(txDutyCycleTime);
      deviceState = DEVICE_STATE_SLEEP;
      break;
    }
    case DEVICE_STATE_SLEEP:
    {
      LoRaWAN.sleep();
      break;
    }
    default:
    {
      deviceState = DEVICE_STATE_INIT;
      break;
    }
  }
}

//Commands the sensor to take a range reading
void takeRangeReading(){
  Wire.beginTransmission(SensorAddress); //Start addressing
  Wire.write(RangeCommand); //send range command
  Wire.endTransmission(); //Stop and do something else now
}

//Returns the last range that the sensor determined in its last ranging cycle in centimeters. Returns 0 if there is no communication.
void requestRange(){
    Wire.requestFrom(SensorAddress, byte(2));
    if(Wire.available() >= 2){ //Sensor responded with the two bytes
      byte HighByte = Wire.read(); //Read the high byte back
      byte LowByte = Wire.read(); //Read the low byte back
      range = (HighByte*256)+LowByte ; //Make a 16-bit word out of the two bytes for the range
      Serial.print("Range: "); 
      Serial.println(range); //Print to the user
    }
    else {
    }
}

void readGPS()
{
  
  while(serial_connection.available()==0 or digitalRead(GPIO0) == 0){
    //waiting for serial read
  }
    
  while(gps.location.isUpdated() == 0)//While there are characters to come from the GPS
  {
    
      while(serial_connection.available())//While there are characters to come from the GPS
       {
          gps.encode(serial_connection.read());//This feeds the serial NMEA data into the library one char at a time   
       }
  }
  Serial.println("DONE READ GPS");
}
