#include <VescUart.h>

VescUart UART;

#define gearRatio 26
#define k_b 11.5
#define r_eff 0.0175
#define I_thresh 13

// Variables for current calculation 
float v;     //velocity [m/s]
float x;     //amplitude [m]
float bp;  //PTO damping 
float kp;  //PTO stiffness
float M;   //mass [kg] for point absorber, mass moment of inertia [kg-m^2] for osc flap
float K;    //stiffness [N/m]
float A;   //added mass [kg[]
float B;   //radiation damping [kg/s]
volatile float theta = 0; 
float c_speed;

// Variables for Rpi UI info
String message;  //message from Rpi
float frequency;   //frequency value
String wec_type; //wec type (PA ore  OS)
float amp;           //amplitude [m]

int freqIndex;     //index of frequency in message
int ampIndex;    //index of amplitude in message
int typeIndex;    //index of wec type in message

// Variables for Forced Osc Test rpm calculation
float v_set = 1.831; //velocity [m/s]
float rpm;                //rpm [revs/minute]
float offTime;          //amount of time motor should be off [ms]
float onTime;          //amount of time motor should be on [ms]
float w_motor;        //angular velocity of motor [rad/sec]

//Test variables
float current = 2.10;
bool oscTest;          //bool value for OSC test
bool motorControl; //bool value for motor control

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  while (!Serial1) {;}

  //Set UART comm to VESC to Serial1
  UART.setSerialPort(&Serial1);
  oscTest = 0;
  motorControl = 0;
}

 void parse_message(String message) {

  
  //Get the index of frequency and type
  freqIndex = message.indexOf("Frequency:");
  typeIndex = message.indexOf("Type:");
  ampIndex = message.indexOf("Amp:");

  //If these imdexes exist then start parsing 
  if(freqIndex != -1 && typeIndex != -1){
    //go past the 10 characters in frequency and get the frequency value
    //stop at the delimiter "|"
    //convert the frequency from a string to int  
    frequency = message.substring(freqIndex + 10, message.indexOf("|", freqIndex)).toInt();
    
    //Obtain wec_type from the rest of the message 
    wec_type = message.substring(typeIndex + 5, message.indexOf("|", typeIndex));
    }

   if( ampIndex != -1) {
    //Obtain the amp information 
    amp = message.substring(ampIndex + 4).toFloat();
    }
    //Serial.println(amp);
  }

  
 void calcOscTestRPM(float amp) {
  //calculate time to spin
  onTime = amp/v_set;

  //omega 
  w_motor = v_set/r_eff;

  //time to stay off
  offTime = (1/frequency) * 1000;

  //calculating rpm 
  rpm = w_motor * 9.55;  //maybe change this into 60/2*pi
  //Serial.println(v_set);
  //Serial.println(r_eff);
  //Serial.println(w_motor);
  //Serial.println(rpm);
 }

 void set_parameters(int frequency, String wec_type){

  if(wec_type == "PA"){
    M = 5.3;
    K = 643.7;

    if(frequency == 1) {
          A = 5.36;
          B = 10.47;
    }

     else if(frequency == 0.832) {
          A = 5.65;
          B = 9.33;
     }

     else if(frequency == 0.719){
          A = 6.13;
          B = 8.081;
        
     }
  
    }

    if(wec_type == "OS"){
    M = 0.0082;
    K = 1636.98;
    
    
    if(frequency == 1) {
          A = 0.0167;
          B = 7.35;
    }

     else if(frequency == 0.832) {
          A = 0.01676;
          B = 2.862;
     }

     else if(frequency == 0.719){
          A = 0.0167;
          B = 1.253;
        
     }
  
    }

    bp = B;
    kp = sqrt(K/(M + A));
  
  }

  float calcCurrent(float w_motor, float thetha, String wec_type) {

    if(wec_type == "PA"){
     v = w_motor * r_eff;
     x = thetha * r_eff;
    }
    else if(wec_type == "OS") {
     v = w_motor * gearRatio;
     x = thetha * gearRatio; 
     }

     float F_desired = kp*x + bp*v;

     float I_desired = F_desired * r_eff * k_b;

     if(abs(I_desired) > I_thresh) {
      I_desired = 0.0;
      }

    return I_desired;
    }

void loop() {
  //Look for serial message from Rpi
  if(Serial.available() > 0) {
          message = Serial.readStringUntil('\n');
  }

  //Stop the motor if the message is "Stop"                                  
  if(UART.getVescValues() && message == "Stop") {
         oscTest = 0;
         motorControl = 0;
         ampIndex = -1;
         freqIndex = -1;
         typeIndex = -1;
         UART.setRPM(0.00);
         UART.setCurrent(0.00);
 }

  //Else parse the message
  else if(message != "Stop" && message){
         parse_message(message);
  }

  //If amplitude is part of the message it is a forced oscillation test
  if(ampIndex != -1) {
        //calculate RPM
        calcOscTestRPM(amp);
        //set OscTest to true
        oscTest = 1; 
   }
 else if(freqIndex != -1 && typeIndex != -1 && ampIndex == -1) {
        motorControl = 1;
   }

   if(oscTest){
       //Serial.println("RPM set");
       UART.setRPM(rpm);
       //delay(onTime);
    
       //UART.setRPM(0.00);
       //delay(offTime);
        
    }

    while(motorControl) {
        // set A, B, M, K 
        set_parameters(frequency, wec_type);
        theta = 2 * 3.14159 * (UART.data.tachometer/126);
        c_speed = (UART.data.rpm/21)*(3.14159265359/60);
      
        UART.setCurrent(calcCurrent(c_speed, theta, wec_type));
        //Serial.println("Motor controls code called");
      }
   
  }
