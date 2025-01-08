// Modified code created during Winter break 2025 
// Still needs to be tested 

#include <ezButton.h>
#include <VescUart.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

#define gearRatio 10.0        // RM5 gear ratio
#define k_b 11.5              // motor constant               [Amp/Nm]   
#define r_eff 0.010/(2*3.14)  // effective radius of the lead screw = lead/2pi           [m]
#define I_thresh 13           // current threshold                [Amp]


float v ;
float x ;
int frequency;                // frequency of the wave
int converterType;            // type of converter (1 = Oscillating, 2 = Point Absorber)
volatile float theta = 0;
float c_speed = 0;
double mass = 0.0;
double added_mass = 0.0;
double radiation_damping = 0.0;
double stiffness = 0.0;
double kp = 0.0;
double bp = 0.0;
 
VescUart UART;

void setup(){
    Serial.begin(9600);
    //For testing purposes, the initial current of the motor is set to 2
    UART.setCurrent(2);
}

void loop(){
    Serial.println("Please enter the frequency value (1,2,3): ");

    while (Serial.available() == 0) {
        // wait for user input
    }

    int frequency = Serial.parseInt();

    Serial.println("Please enter the converter type (1 = osc, 2 = point): ");

    while (Serial.available() == 0) {
        // wait for user input
    }

    int converterType = Serial.parseInt();

    set_parameters(frequency, converterType);

    if (UART.getVescValues()){
        c_speed = (UART.data.rpm/21)*(3.14159265359/60);
        theta = 2 * 3.14159 * (UART.data.tach/126);
        UART.setCurrent(calcVoltage(1,c_speed, 2, theta, 3));
        UART.printVescValues();
    }
    else{
        Serial.println("Failed to get VESC values");
    } 
}


void set_parameters(int frequency, int converterType) {

    if (converterType == 1){
        switch (frequency) {
            case 1:
                mass = 10.0;
                added_mass = 2.0;
                radiation_damping = 1.0;
                stiffness = 200.0;
                kp = 0.5;
                bp = 0.1;
                break;
            case 2:
                mass = 12.0;
                added_mass = 2.5;
                radiation_damping = 1.2;
                stiffness = 250.0;
                kp = 0.6;
                bp = 0.2;
                break;
            case 3:
                mass = 15.0;
                added_mass = 3.0;
                radiation_damping = 1.5;
                stiffness = 300.0;
                kp = 0.7;
                bp = 0.3;
                break;
            default:
                Serial.println("Invalid frequency type for Oscillating Water Column");
                return;
        }
    }
    else if (converterType == 2){
        switch (frequency) {
            case 1:
                mass = 5.0;
                added_mass = 1.0;
                radiation_damping = 0.5;
                stiffness = 100.0;
                kp = 0.3;
                bp = 0.05;
                break;
            case 2:
                mass = 6.0;
                added_mass = 1.5;
                radiation_damping = 0.6;
                stiffness = 150.0;
                kp = 0.4;
                bp = 0.1;
                break;
            case 3:
                mass = 7.0;
                added_mass = 2.0;
                radiation_damping = 0.7;
                stiffness = 200.0;
                kp = 0.5;
                bp = 0.15;
                break;
            default:
                Serial.println("Invalid frequency type for Point Absorber");
                return;
        }
    }
    else{
        Serial.println("Invalid converter type");
        return;
    }
}
    
//need to add white noise function to send signal to motor, collect data on power being produced

float calcVoltage(float I, float w_motor, float w_wave, float theta, int RMtype) {
  if (RMtype = 3) {
    v = w_motor * r_eff;  // instead of r_eff, r_pin (radius of rpin is hardcode number, find out)
    x = theta   * r_eff;
  } else if (RMtype = 5) {
    v = w_motor * gearRatio;  
    x = theta   * gearRatio;
  }
  float F_desired = kp*x + bp*v;
  float I_desired = F_desired * r_eff * k_b;
  //Serial.print("kp*x = ");
  //Serial.println(kp*x);
  //Serial.print("bp*v = ");
  //Serial.println(bp*v);
  //Serial.print("I_desired = ");
  //Serial.println(I_desired);
  if (abs(I_desired) > I_thresh) {
    I_desired = 0.0; // if spinning uncontrollably, stop commanding force
  }
  return I_desired;
}
