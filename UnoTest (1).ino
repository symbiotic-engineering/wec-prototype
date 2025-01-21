#include <VescUart.h>

VescUart UART;

float current = 2.10;

String message;

int frequency;
String wec_type;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  while (!Serial1) {;}

  UART.setSerialPort(&Serial1);

}

void loop(){

  //Look for serial message from Rpi
  if(Serial.available() > 0) {
          message = Serial.readStringUntil('\n');
  }

  //Stop the motor if the message is "Stop"
   if(UART.getVescValues() && message == "Stop") {
          frequency = 0;
          UART.setCurrent(0.00);
    }
   //Otherwise parse the message
   else{
    parse_message(message);
    }

  //For testing
  if(frequency == 1 && wec_type == "OS"){
    UART.setCurrent(1.00);
    }
   if(frequency ==2 && wec_type == "PA"){
    UART.setCurrent(3.00);
    }

}

void parse_message(String message) {

  //Get the index of frequency and type
  int freqIndex = message.indexOf("Frequency:");
  int typeIndex = message.indexOf("Type:");

  //If these imdexes exist then start parsing 
  if(freqIndex != -1 && typeIndex != -1){
    //go past the 10 characters in frequency and get the frequency value
    //stop at the delimiter "|"
    //convert the frequency from a string to int  
    frequency = message.substring(freqIndex + 10, message.indexOf("|", freqIndex)).toInt();
    
    //Obtain wec_type from the rest of the message 
    wec_type = message.substring(typeIndex + 5);
    }

  }
