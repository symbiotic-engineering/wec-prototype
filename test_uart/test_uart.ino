/*
 Name:		VescUartSample.ino
 Created:	9/26/2015 10:12:38 PM
 Author:	AC
*/

// the setup function runs once when you press reset or power the board
// To use VescUartControl stand alone you need to define a config.h file, that should contain the Serial or you have to comment the line
// #include Config.h out in VescUart.h

//Include libraries copied from VESC
#include "VescUart.h"
#include "datatypes.h"
#include "Arduino.h"
#include "HardwareSerial.h"
#include "Config.h"

//const int PB10 = 29;
//const int PB11 = 30;


//HardwareSerial Serial3(PB11,PB10);

//#define DEBUG
unsigned long count = 0;
struct bldcMeasure measuredValues;
	

void setup() {
	
  Serial3.setRx(PB11);
  Serial3.setTx(PB10);
  //pinMode(PC1,OUTPUT);

	//Setup UART port
	Serial3.begin(115200);
  #ifdef DEBUG
	//SEtup debug port
	Serial.begin(115200);
  #endif
//Serial.print(": ");
}


// the loop function runs over and over again until power down or reset
void loop() {
	int len=0;
  uint8_t message[256];
	len = ReceiveUartMessage(message,3);
  
	// if (len > 0)
	// {
	// 	len = PackSendPayload(message, len,3);
  //   Serial.println("On loop"); Serial.print(count); Serial.println(" lenPayload is "); Serial.println( len);
	// 	len = 0;
	// }
  
//	Serial.print("Loop: "); Serial.println(count++);
  
	if (VescUartGetValue(measuredValues)) {
    Serial.print("Got values");
		Serial.print("Loop: "); Serial.println(count++);
    Serial.print("Current is: ");
		Serial.println(measuredValues.avgInputCurrent);
    //digitalWrite()
     //digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(10000);  
                   // wait for a second
  //digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
   // Serial.println(measuredValues);
    // delay(1000);    
	}
	else
	{
		Serial.println("Failed to get data!");
    //  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(10000);                      // wait for a second
  
	}
  
	
}
