#include "LoRaWan_APP.h"
#include "Arduino.h"

#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS GPIO2

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

/* OTAA para*/
uint8_t devEui[] = { 0xab, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; //ABC0000000000000
uint8_t appEui[] = { 0xab, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; //"ABC0000000000000"
uint8_t appKey[] = { 0x16, 0x28, 0xae, 0x2b, 0x7e, 0x15, 0xd2, 0xa6, 0xab, 0xf7, 0xcf, 0x4f, 0x3c, 0x15, 0x88,0x09 };  //"1628AE2B7E15D2A6ABF7CF4F3C158809"
/* ABP para*/
uint8_t nwkSKey[] = { 0x28, 0xae, 0xd2, 0x2b, 0x7e, 0x15, 0x16, 0xa6, 0x09, 0xcf, 0xab, 0xf7, 0x15, 0x88, 0x4f,0x3c };     //"28AED22B7E1516A609CFABF715884F3C"
uint8_t appSKey[] = { 0x16, 0x28, 0xae, 0x2b, 0x7e, 0x15, 0xd2, 0xa6, 0xab, 0xf7, 0xcf, 0x4f, 0x3c, 0x15, 0x88,0x09 };     //"1628AE2B7E15D2A6ABF7CF4F3C158809"
uint32_t devAddr =  ( uint32_t )0x007ab001;

/*LoraWan channelsmask, default channels 0-7*/ 
uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 60000*10;

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
    unsigned char *puc1,*puc2,*puc3,*puc4,*puc5,*puc6,*pucbat ;
    delay(500);
    
    sensors.requestTemperatures();
    int Celsius = sensors.getTempCByIndex(0)*10;
    int voltage = getBatteryVoltage()/10;
    
    Serial.print("Voltage =");
    Serial.println(voltage);

    Serial.print("  >> Temp water:");
    Serial.print(float(sensors.getTempCByIndex(0)));
    Serial.print(" Cº  //  \n  ");
    

    puc1 = (unsigned char *)(&Celsius);
    puc2 = (unsigned char *)(&voltage);
        
    appDataSize = 8; 
 
    appData[0] = 0x00;
    appData[1] = 0x67;
    appData[2] = puc1[1];
    appData[3] = puc1[0];
    
    appData[4] = 0x01;
    appData[5] = 0x02;
    appData[6] = puc2[1];
    appData[7] = puc2[0];
    
    digitalWrite(Vext, HIGH);  //Close Vext
    Serial.println("complete prepare");
}


void setup() {
  boardInitMcu();
  Serial.begin(115200);
  pinMode(Vext, OUTPUT); //Open Vext
  sensors.begin();
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
