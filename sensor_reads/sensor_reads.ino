// Power System Sensor Reading and State Machine Module
// 
// This module is split into two sections: the function d

// ---- Importing Libraries -----

//#include "vesc_uart.h" <- OLD LIBRARY

#include <VescUart.h>

// ---- Definitions -----

/** Initiate VescUart class */
VescUart UART;

// We are creating a struct (short for structure) called vesc_reading, to hold all the values
// embedded in the VESC's output
struct vesc_reading{
    float current = 0.0;           //measured battery current
    float motor_current = 0.0;     //measured motor current
    float voltage = 0.0;           //measured battery voltage
    float c_speed = 0.0;           //measured rpm * Pi * wheel diameter [km] * 60 [minutes]
    float c_dist = 0.00;           //measured odometry tachometer [turns] * Pi * wheel diameter [km] 
    double power = 0.0;              //calculated power
};


//---------------------------------------------------------------------------------------------
// VARIABLES
//---------------------------------------------------------------------------------------------

//ASSIGNING VARIABLES TO EACH OF THE PIN CONNECTIONS
//const int hallsensorPin; //= pin number of the hall_sensor
const int battvoltsensorPin = 24; //= pin number of the battery voltage sensor
const int battcurrentsensorPin = 25; //= pin number of the battery current sensor
const int wavegaugePin = 23; //= pin number of the wave gauge
const int torquesensorPin = 21; //=pin number of the torque sensor



//Vin/Vout https://cdn.sparkfun.com/assets/c/a/a/4/6/Voltage_to_Voltage_45a.png reduction factor of the Voltage Sensor Shield
const float factor = 4.16; 

//input voltage for sensor
const float vCC = 13.6; 




//---------------------------------------------------------------------------------------------
// HELPER FUNCTIONS
//---------------------------------------------------------------------------------------------

// We define functions to 
      // 1. Read sensors
      // 2. Detect Faults
      // 3. Run the state machine



//------------------------------------------------
// Sensor Reading Functions 
//------------------------------------------------

// The power system has 5 total sensors.


// -------------------------------- vesc -----------

vesc_reading vesc_read(){
  //Returns: a parsed, UART reading, type vesc_reading (the struct we defined at the beginning)

  //float reading_hall = analogRead(hallsensorPin);
  struct vesc_reading vesc_val;

  if (UART.getVescValues()) {
      
    // calculation of several values to be displayed later on
    // See https://github.com/SolidGeek/VescUart/blob/master/src/VescUart.cpp for uart readings
    vesc_val.current = UART.data.ampHours;
    vesc_val.voltage = UART.data.inpVoltage;
      
    vesc_val.motor_current = UART.data.avgMotorCurrent;
        
    vesc_val.c_speed = (UART.data.rpm/38)*3.14159265359*0.000083*60;
    vesc_val.c_dist = (UART.data.tachometerAbs/38)*3.14159265359*0.000083;
        
    vesc_val.power = vesc_val.current*vesc_val.voltage;
  }
  else
  {
    Serial.println("Failed to get data!");
  }

  return vesc_val;
}


// -------------------------------- battery voltage -----------
// reference: https://github.com/BasOnTech/Arduino-Beginners-EN/blob/master/E17-voltage-sensor/voltage-sensor.ino
float batt_voltage_read(){
  // Returns: voltage value, type float
  float analog_reading = analogRead(battvoltsensorPin)/1024; //the 1024 is included because that is the number of int in 10 bits
  float V_adc = analog_reading*3.3; //voltage level of microcontroller pin
  float V_sensor = V_adc * 18.18/5; //the 15/5 is included because a voltage divider with R1= 13.18 kohms and R2=5 kohms is used to bring down sensor voltage of 12V to 3.3V
  float batt_voltage_val = V_sensor * 4.1379; //https://cdn.sparkfun.com/assets/c/a/a/4/6/Voltage_to_Voltage_45a.png
  Serial.print("Voltage = ");
  Serial.println(batt_voltage_val);
  
  return batt_voltage_val;

}

// -------------------------------- battery current -----------
float batt_current_sensor_read(){
  // Returns: a current value, type float
  float analog_reading = analogRead(battcurrentsensorPin)/1024; //the 1024 is included because that is the number of int in 10 bits
  float V_adc = analog_reading*3.3; //voltage level of microcontroller pin
  float batt_current_val = V_adc * 13.67; //https://cdn.sparkfun.com/assets/8/a/9/4/b/Current_to_Voltage_45a.png

  Serial.print("Source current= ");
  Serial.println(batt_current_val);

  return batt_current_val;
}

// -------------------------------- wave gauge -----------

float wave_gauge_read(){
  //Returns: wave height value, type float

  float k = 25.7069; //conversion factor found experimentally
  float analog_reading = analogRead(wavegaugePin)/1024; //the 1024 is included because that is the number of int in 10 bits
  float V_adc = analog_reading * 3.3; //voltage level of microcontroller pin
  float V_sensor = V_adc * 15/5; //the 15/5 is included because a voltage divider with R1= 10 kohms and R2=5 kohms is used to bring down sensor voltage of 10V to 3.3V
  float wave_gauge_val = k*V_sensor+31.619487; //y-intercept of 31.619487

  Serial.print("Wave gauge = ");
  Serial.println(wave_gauge_val);
  

  return wave_gauge_val;
}


// -------------------------------- torque sensor -----------

float torque_sensor_read(){
  // Returns: torque value, type float
 
  float k = 1.0361; //conversion factor found experimentally, note that there is also a y intercept in the graph
  float analog_reading = analogRead(torquesensorPin)/1024; //the 1024 is included because that is the number of int in 10 bits
  float V_adc = analog_reading * 3.3; //voltage level of microcontroller pin
  float V_sensor = V_adc * 15/10; //the 15/10 is included because a voltage divider with R1= 5 kohms and R2=10 kohms is used to bring down sensor voltage of 5V to 3.3V
  float torque_val = k*V_sensor+0.1745; //y-intercept of 0.1745

  Serial.print("Torque = ");
  Serial.println(torque_val);

  return torque_val;
}


//------------------------------------------------
// 2. Check Faults
//------------------------------------------------

// char check_faults (struct vesc_reading* vesc_val){
//   // Returns: the kind of fault (soft_fault / hard_fault / fatal_fault) if fault, otherwise default is "none"
//   // Parameter vesc_val: the output value of the vesc, type "struct vesc_reading" (we defined the vesc_reading structure in line 9)

//       // Note that we add the asterisk (*) because the parameter is in reality the pointer to the memory location of the 
//       //      structure's first value. C is very low level and you have to pass on where things are located in memory 

  
//   char fault_type = "none";
//   return fault_type;

// }


//------------------------------------------------
// 3. State Machine 
//------------------------------------------------

// Diagram available at https://tinyurl.com/7mwzab2m
// char run_state_machine(char fault_type, int wave_gauge_val, int torque_val, int batt_voltage_val, int batt_current_val, struct vesc_reading* vesc_val){
//   // Returns: the state machine's current state

//   // Paramenter fault_type: the type of fault, type char
//   // Paramenter wave_gauge_val: the height of the wave, type int
//   // Paramenter torque_val: the torque value read by the torque sensor, type int
//   // Paramenter batt_voltage_val: the battery voltage value as read by the voltage/current sensor, type int
//   // Paramenter batt_current_val: the battery current value as read by the voltage/current sensor, type int
//   // Paramenter vesc_val: the output value of the vesc, type "struct vesc_reading" (we defined the vesc_reading structure in line 9)

//       // Note that we add the asterisk (*) because the parameter is in reality the pointer to the memory location of the 
//       //      structure's first value. C is very low level and you have to pass on where things are located in memory 


//   // ---------------------  State Definitions  --------------------

//   const int STATE_ROOT                    = 0;
//   const int STATE_START_PRECHARGE         = 1;
//   const int STATE_PRECHARGING_I           = 2;
//   const int STATE_PRECHARGING_II          = 3;
//   const int STATE_PRECHARGE_DONE          = 4;
//   const int STATE_SWITCHING               = 5;
//   const int STATE_REFILL_WAIT             = 6;
//   const int STATE_TORQUING                = 7;

//   // ------------------------  State Logic  -----------------------

//   // Writes the logic to transition to the next state
//   volatile state_current;
//   volatile state_next;

//   if (e_stop){
//     state_current = STATE_ROOT;
//   }
//   else {
//     state_current = state_next;
//   }

//   // ---------------------  State Transitions  --------------------

//   // Fault trackers
//   volatile bool faults;
//   volatile bool solft_fault;
//   volatile bool hard_fault;
//   volatile bool fatal_fault;

//   // True if the state machine passed throug the precharging process
//   // False if there is no need to precharge again (hard & soft fault)

//   volatile bool it_comes_from_HV;

//   // Botton trackers
//   volatile bool advance;
//   volatile bool e_stop;

//   //advance = 




//   // -----------------------  State Outputs  ----------------------

//   return state_current
// }



//---------------------------------------------------------------------------------------------
// VOID SETUP
//---------------------------------------------------------------------------------------------
void setup() {
  // pinMode(hallsensorPin, INPUT);
  pinMode(battvoltsensorPin, INPUT);
  pinMode(battcurrentsensorPin, INPUT);
  pinMode(wavegaugePin, INPUT);
  pinMode(torquesensorPin, INPUT);
  Serial.begin(9600);

  /** Setup UART port (Serial1 on Atmega32u4) */
  Serial3.begin(115200);
  
  while (!Serial) {;}

  /** Define which ports to use as UART */
  UART.setSerialPort(&Serial3);
}


//---------------------------------------------------------------------------------------------
// VOID LOOP
//---------------------------------------------------------------------------------------------
void loop() {

  //Read sensors
  float batt_voltage_val = batt_voltage_read();
  float batt_current_val = batt_current_sensor_read();
  float wave_gauge_val = wave_gauge_read();
  float torque_val = torque_sensor_read();
  struct vesc_reading vesc_val = vesc_read();

  //check_faults();
  //run_state_machine();

  
}
