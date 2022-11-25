// Power System Sensor Reading and State Machine Module
// 
// This module is split into two sections: the function d


//---------------------------------------------------------------------------------------------
// VARIABLES
//---------------------------------------------------------------------------------------------

//ASSIGNING VARIABLES TO EACH OF THE PIN CONNECTIONS
const int hallsensorPin; //= pin number of the hall_sensor
const int battvoltsensorPin; //= pin number of the battery voltage sensor
const int battcurrentsensorPin; //= pin number of the battery current sensor
const int wavegaugePin; //= pin number of the wave gauge
const int torquesensorPin; //=pin number of the torque sensor

//ASSIGNING VARIABLES FOR THE READINGS OF EACH OF THE PINS
int reading_hall_sensor;

//value on pin
float reading_voltage; 

// measured voltage 
float vIn; 

float vOut;

//Vin/Vout https://cdn.sparkfun.com/assets/c/a/a/4/6/Voltage_to_Voltage_45a.png reduction factor of the Voltage Sensor Shield
const float factor = 4.16; 

//input voltage for sensor
const float vCC = 13.6; 


float reading_current;
float current;

int reading_wave_gauge;
int reading_torque_sensor;

// -------------------- VOID SET UP --------------------
void setup() {
pinMode(hall_sensor, INPUT);
pinMode(batt_volt_sensor, INPUT);
pinMode(batt_current_sensor, INPUT);
pinMode(wave_gauge, INPUT);
pinMode(torque_sensor, INPUT);
Serial.begin(9600);
}


//---------------------------------------------------------------------------------------------
// Sensor Reading Functions 
//---------------------------------------------------------------------------------------------

// The power system has 5 total sensors.

void hall_sensor_read(){
//NEEDS TO BE DONE
  reading_hall = analogRead(hallsensorPin)
}
void batt_voltage_read(){
  reading_voltage = analogRead(battvoltsensorPin);
  vOut = (reading_voltage/1024)*vCC; //read the current sensor value (0-1023)
  //We might need a voltage divider if we want max voltage to be 3.3 int he case of voltage measurements
  vIn = vOut*factor:
  Serial.print("Voltage = ");
  Serial.print(vIn);
  Serial.println("V");
  // reference: https://github.com/BasOnTech/Arduino-Beginners-EN/blob/master/E17-voltage-sensor/voltage-sensor.ino 
}
void batt_current_sensor_read(){
  reading_current = analogRead(battcurrentsensorPin);
  current = (reading_current/1024)*3.3; //https://cdn.sparkfun.com/assets/8/a/9/4/b/Current_to_Voltage_45a.png
  
  Serial.print("Source current= ");
  Serial.print(current);
}
void wave_gauge_read(){
  //NEEDS TO BE DONE
  reading_wave_gauge = analogRead(wavegaugePin)
}

void torque_sensor_read(){
  //NEEDS TO BE DONE - I think it needs to be calibrated manually because there is no data sheet for it. For this you use a known inertia in the output, accelerate at a know veloctiy and use Newton's II law torque = inertia*acceleration and measured the voltage output.
  //Measures torque from 0.5-150Nm and has an output signal of 0-20mA (not sure what capacity the one being used has)
  //https://www.ato.com/micro-reaction-torque-sensor-0d5-nm-to-150-nm
  //looks like it is connected to two pins
  
  reading_torque_sensor = analogRead(torquesensorPin)
  //insert our conversion factor here
}

//---------------------------------------------------------------------------------------------
// Finite State Machine 
//---------------------------------------------------------------------------------------------

void loop() {
  // put your main code here, to run repeatedly:

  // ---------------------  State Definitions  --------------------

  // ------------------------  State Logic  -----------------------

  // ---------------------  State Transitions  --------------------

  // -----------------------  State Outputs  ----------------------
  

}
