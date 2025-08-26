#include <VescUart.h>

VescUart UART;

float current = 2.10;
float rpm = 0.00;
#define r_eff 17.5

//Variables for current calculation
float off;
float v = 0.05;
float t;
float  w_motor;

String message;

//Variables coming from Rpi UI
float frequency;
String wec_type;
float amp;

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
          frequency = 0.00;
          amp = 0.00;
          rpm = 0.00;
          UART.setCurrent(0.00);
    }
   //Otherwise parse the message
   else{
    parse_message(message);
    calcCurrent(amp);
   }

   //UART.setRPM(rpm);

   //Serial.print(rpm);

  //For testing
  //if(amp == 0.06){
  //UART.setCurrent(1.00);
  //}
}

void parse_message(String message) {

  //Get the index of frequency and type
  int freqIndex = message.indexOf("Frequency:");
  int typeIndex = message.indexOf("Type:");
  int ampIndex = message.indexOf("Amp:");

  //If these imdexes exist then start parsing 
  if(freqIndex != -1 && typeIndex != -1 && ampIndex != -1){
    //go past the 10 characters in frequency and get the frequency value
    //stop at the delimiter "|"
    //convert the frequency from a string to int  
    frequency = message.substring(freqIndex + 10, message.indexOf("|", freqIndex)).toInt();
    
    //Obtain wec_type from the rest of the message 
    wec_type = message.substring(typeIndex + 5, message.indexOf("|", typeIndex));

    //Obtain the amp information 
    amp = message.substring(ampIndex + 4).toFloat();
    }
  }

 void calcCurrent(float amp) {
  //calculate time to spin
  t = amp/v;

  //omega 
  w_motor = v/r_eff;

  //time to stay off
  off = 1/frequency;

  //calculating rpm 
  rpm = w_motor * (60/2*3.14);

  Serial.print(rpm);
  
 }
