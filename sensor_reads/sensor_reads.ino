// Power System Sensor Reading and State Machine Module
// 
// This module is split into two sections: the function d

// ---- Importing Libraries -----

//#include "vesc_uart.h" <- OLD LIBRARY

#include <VescUart.h>
#include <ezButton.h>
#include <TimerOne.h>

// ---- Definitions -----

/** Initialize VescUart class */
VescUart UART;

/* FSM Variables */
/* Initialize FSM Global variables */
int fault = 0;
int bat_voltage = 0;
int advance = 0;

enum State {ROOT, PRECHARGE, HIGH_VOLTAGE, SWITCH_WAIT, TORQUING, SWITCH_AUTO};
enum State current_state = ROOT;

/* FSM Testing Config */
ezButton FATAL_FAULT(2);
ezButton HARD_FAULT(4);
ezButton SOFT_FAULT(7);
ezButton ADVANCE(8);
ezButton BAT_VOLTAGE(12);

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
  float voltage_reading = analogRead(battvoltsensorPin);
  float vOut = (voltage_reading/1024)*vCC; //read the current sensor value (0-1023) 
  //We might need a voltage divider if we want max voltage to be 3.3 int he case of voltage measurements
  float batt_voltage_val = vOut*factor;
  Serial.print("Voltage = ");
  Serial.println(batt_voltage_val);
  
  return batt_voltage_val;

}

// -------------------------------- battery current -----------
float batt_current_sensor_read(){
  // Returns: a current value, type int
  float current_reading = analogRead(battcurrentsensorPin);
  float batt_current_val = (current_reading/1024)*3.3; //https://cdn.sparkfun.com/assets/8/a/9/4/b/Current_to_Voltage_45a.png
  
  Serial.print("Source current= ");
  Serial.println(batt_current_val);

  return batt_current_val;
}

// -------------------------------- wave gauge -----------

float wave_gauge_read(){
  //Returns: wave height value, type float

  float k = 25.7069; //conversion factor found experimentally, note that there is also a y intercept in the graph with value 31.619487
  float wave_gauge_reading = analogRead(wavegaugePin)/1024; //the 1024 is included because that is the number of int in 10 bits
  float V_adc = wave_gauge_reading * 3.3;
  float V_sensor = V_adc * 15/5; //the 15/5 is included because a voltage divider with R1= 10 kohms and R2=5 kohms is used to bring down Vcc of 10V to 3.3V
  float wave_gauge_val = k*V_sensor+31.619487; 

  Serial.print("Wave gauge = ");
  Serial.println(wave_gauge_val);
  

  return wave_gauge_val;
}


// -------------------------------- torque sensor -----------

float torque_sensor_read(){
  // Torque sensor conversion ratio is k

  // Returns: torque value, type float
 
  float k = 1.0361; //conversion factor found experimentally, note that there is also a y intercept in the graph
  float analog_reading = analogRead(torquesensorPin)/1024; //the 1024 is included because that is the number of int in 10 bits
  float V_adc = analog_reading * 3.3;
  float V_sensor = V_adc * 15/10; //the 15/10 is included because a voltage divider with R1= 5 kohms and R2=10 kohms is used to bring down Vcc of 5V to 3.3V
  float torque_val = k*V_sensor+0.1745;
  //the 0.1745 is included because that's where the torque sensor was zeroed

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




//   // -----------------------  State Outputs  ----------------------

//   return state_current
// }
/**
 * @brief Checks flags to perform FSM state transitions.
 * Will modify advance global variable.
 */
void state_machine(){ //TODO take in valid input for current_state transitions
  switch(current_state){
    case ROOT: // (ROOT->PRECHARGE) advance && !faults
      if(advance == 1 && fault == 0/*advance && !faults*/){
        // advance = 0;
        current_state = PRECHARGE;
        Serial.println(current_state);
      }
      break;
    case PRECHARGE: // (PRECHARGE->HIGH_VOLTAGE) !faults && bat_voltage_is_high_enough
      if(fault == 0 && bat_voltage == 1/*!faults && bat_voltage_is_high_enough*/){
        current_state = HIGH_VOLTAGE;
        Serial.println(current_state);
      }
      break;
    case HIGH_VOLTAGE: // (HIGH_VOLTAGE->SWITCH_WAIT) advance && !faults
      if(advance == 1 && fault == 0/*advance && !faults*/){
        current_state = SWITCH_WAIT;
        Serial.println(current_state);
      }
      break;
    case SWITCH_WAIT: // (SWITCH_WAIT->TORQUING) advance && !faults
      if(advance == 1 && fault == 0/*advance && !faults*/){
        current_state = TORQUING;
        Serial.println(current_state);
      }
      break;
    case TORQUING: // (TORQUING->TORQUING) !faults (TORQUING->ROOT) fatal_fault (TORQUING->PRECHARGE_DONE) hard_fault (TORQUING->SWITCH_AUTO) soft_fault
      if(fault == 3/*fatal_fault*/){
        current_state = ROOT;
        Serial.println(current_state);
      }
      else if(fault == 2/*hard_fault*/){
        current_state = HIGH_VOLTAGE;
        Serial.println(current_state);
      }
      else if(fault == 1/*soft_fault*/){
        current_state = SWITCH_AUTO;
        Serial.println(current_state);
      }      
      break;
    case SWITCH_AUTO: // (SWITCH_AUTO->TORQUING)!faults
      if(fault == 0/*!faults*/){
        current_state = TORQUING;
        Serial.println(current_state);
      }
      break;
  }
  advance = 0;
}

/**
 * @brief Simulate FSM behavior using button inputs
 * 
 */
void simulate_fsm_behavior(){
  FATAL_FAULT.loop(); // MUST call the loop() function first
  HARD_FAULT.loop(); // MUST call the loop() function first
  SOFT_FAULT.loop(); // MUST call the loop() function first
  ADVANCE.loop(); // MUST call the loop() function first
  BAT_VOLTAGE.loop(); // MUST call the loop() function first

  if (FATAL_FAULT.isPressed()){
    fault = 3;
    // Serial.println("Fatal Fault switch: OFF -> ON");
    // Serial.println(fault);
  }

  if (FATAL_FAULT.isReleased()){
    fault = 0;
    // Serial.println("Fatal Fault switch: ON -> OFF");
    // Serial.println(fault);
  }

  if (HARD_FAULT.isPressed()){
    fault = 2;
    // Serial.println("Hard Fault switch: OFF -> ON");
    // Serial.println(fault);
  }
    

  if (HARD_FAULT.isReleased()){
    fault = 0;
    // Serial.println("Hard Fault switch: ON -> OFF");
    // Serial.println(fault);
  }

  if (SOFT_FAULT.isPressed()){
    fault = 1;
    // Serial.println("Soft Fault switch: OFF -> ON");
    // Serial.println(fault);
  }

  if (SOFT_FAULT.isReleased()){
    fault = 0;
    // Serial.println("Soft Fault switch: ON -> OFF");
    // Serial.println(fault);
  }

  //if (ADVANCE.isPressed())
  //  Serial.println("Advance Button: OFF -> ON");

  if (ADVANCE.isReleased()){
    if (advance == 0){
      advance = 1;
    }
    else {
      advance = 0;
    }
    // Serial.println("Advance Button: ON -> OFF");
    // Serial.println(advance);
  }   

  if (BAT_VOLTAGE.isPressed()){
    bat_voltage = 1;
    // Serial.println("Battery Voltage switch: OFF -> ON");
    // Serial.println(bat_voltage);
  }

  if (BAT_VOLTAGE.isReleased()){
    bat_voltage = 0;
    // Serial.println("Battery Voltage switch: ON -> OFF");
    // Serial.println(bat_voltage);
  }
}


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

  /** FSM code integration */
  // initialize Timer ISR to check for state update
  Timer1.initialize(100000);    
  Timer1.attachInterrupt(state_machine);

  FATAL_FAULT.setDebounceTime(50); // set debounce time to 50 milliseconds
  HARD_FAULT.setDebounceTime(50);
  SOFT_FAULT.setDebounceTime(50);
  ADVANCE.setDebounceTime(50);
  BAT_VOLTAGE.setDebounceTime(50);
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

  simulate_fsm_behavior();
  //check_faults();
  //run_state_machine();

  
}


