// Power System Sensor Reading and State Machine Module
// 
// This module is split into two sections: the function d

// ---- Importing Libraries -----

//#include "vesc_uart.h" I dont have this library 

// ---- TEST LIBRARIES ---- git clone "https://github.com/mmurdoch/arduinounit.git"
#include <ArduinoUnit.h>
#include <ArduinoUnitMock.h>

// ---- Definitions -----
#define MOTOR_SPEED_HARD_THRESH_RPM = 4000;
#define MOTOR_SPEED_SOFT_THRESH_RPM = 3900;
#define RM3_POSITION_SOFT_THRESH_IN = 3.5;
#define RM3_POSITION_HARD_THRESH_IN = 4.5;
#define RM5_POSITION_SOFT_THRESH_DEG = 70;
#define RM5_POSITION_HARD_THRESH_DEG = 80;

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
const int hallsensorPin; //= pin number of the hall_sensor
const int battvoltsensorPin; //= pin number of the battery voltage sensor
const int battcurrentsensorPin; //= pin number of the battery current sensor
const int wavegaugePin; //= pin number of the wave gauge
const int torquesensorPin; //=pin number of the torque sensor



//Vin/Vout https://cdn.sparkfun.com/assets/c/a/a/4/6/Voltage_to_Voltage_45a.png reduction factor of the Voltage Sensor Shield
const float factor = 4.16; 

//input voltage for sensor
const float vCC = 13.6; 



//---------------------------------------------------------------------------------------------
// HELPER FUNCTIONS
//---------------------------------------------------------------------------------------------

// We define functions to 
      // 1. Read sensors - not here
      // 2. Detect Faults
      // 3. Run the state machine
      // 4. Test the above functions

//------------------------------------------------
// 2. Check Faults
//------------------------------------------------

int check_faults (struct vesc_reading* vesc_val){
  // Returns: the kind of fault (soft: 1 / hard: 2 / fatal: 3) if fault, otherwise default is 0
  // Parameter vesc_val: the output value of the vesc, type "struct vesc_reading" (we defined the vesc_reading structure in line 9)

      // Note that we add the asterisk (*) because the parameter is in reality the pointer to the memory location of the 
      //      structure's first value. C is very low level and you have to pass on where things are located in memory 


4.5" for hard fault
3.5" for soft fault

  int fault_type = 0;
  if (vesc_val->current >= 10){
    fault_type = 3;
    return fault_type;
  }

// position fault
if (WHICH_RM == 5 & abs(position) > 70 | WHICH_RM == 3 & abs(position) > 3.5)

// motor current fault
  if (vesc_val->motor_current >= 10){
    motor_fault_type = 3;
  }
  else if (vesc_val->motor_current >= 5){
    motor_fault_type = 2;
    return motor_fault_type;
  }
  else if (vesc_val->motor_current >= 2){
    motor_fault_type = 1;
  }
  else {
    motor_fault_type = 0 
  }

 // motor speed fault
 if (vesc_val->motor_speed >= MOTOR_RPM_SOFT_THRESH & vesc_val->motor_speed < MOTOR_RPM_HARD_THRES){
    motor_fault_type = 1; 
 }
 else if (vesc_val->motor_speed > MOTOR_HARD_THRES){
    motor_fault_type = 2;
 }
 else {
    motor_fault_type = 0;
 }

fault_type = max(motor_fault_type, voltage_fault_type);
return fault_type;
  
  if (vesc_val->voltage >= 10){
    fault_type = 3;
    return fault_type;
  }

  if (vesc_val->c_speed >= 10){
    fault_type = 3;
    return fault_type;
  }

  if (vesc_val->c_dist >= 10){
    fault_type = 3;
    return fault_type;
  }
  if (vesc_val->power >= 10){
    fault_type = 3;
    return fault_type;
  }
  if (vesc_val->current >= 5){
    fault_type = 2;
    return fault_type;
  }



  if (vesc_val->voltage >= 5){
    fault_type = 2;
    return fault_type;
  }

  if (vesc_val->c_speed >= 5){
    fault_type = 2;
    return fault_type;
  }

  if (vesc_val->c_dist >= 5){
    fault_type = 2;
    return fault_type;
  }

  if (vesc_val->power >= 5){
    fault_type = 2;
    return fault_type;
  }
  if (vesc_val->current >= 2){
    fault_type = 1;
    return fault_type;
  }



  if (vesc_val->voltage >= 2){
    fault_type = 1;
    return fault_type;
  }

  if (vesc_val->c_speed >= 2){
    fault_type = 1;
    return fault_type;
  }

  if (vesc_val->c_dist >= 2){
    fault_type = 1;
    return fault_type;
  }

  if (vesc_val->power >= 2){
    fault_type = 1;
    return fault_type;
  }
  
  
  return fault_type;

}


//------------------------------------------------
// 3. State Machine 
//------------------------------------------------

// Diagram available at https://tinyurl.com/7mwzab2m
char run_state_machine(char fault_type, int wave_gauge_val, int torque_val, int batt_voltage_val, int batt_current_val, struct vesc_reading* vesc_val){
  // Returns: the state machine's current state

  // Paramenter fault_type: the type of fault, type char
  // Paramenter wave_gauge_val: the height of the wave, type int
  // Paramenter torque_val: the torque value read by the torque sensor, type int
  // Paramenter batt_voltage_val: the battery voltage value as read by the voltage/current sensor, type int
  // Paramenter batt_current_val: the battery current value as read by the voltage/current sensor, type int
  // Paramenter vesc_val: the output value of the vesc, type "struct vesc_reading" (we defined the vesc_reading structure in line 9)

      // Note that we add the asterisk (*) because the parameter is in reality the pointer to the memory location of the 
      //      structure's first value. C is very low level and you have to pass on where things are located in memory 


  // ---------------------  State Definitions  --------------------

  const int STATE_ROOT                    = 0;
  const int STATE_START_PRECHARGE         = 1;
  const int STATE_PRECHARGING_I           = 2;
  const int STATE_PRECHARGING_II          = 3;
  const int STATE_PRECHARGE_DONE          = 4;
  const int STATE_SWITCHING               = 5;
  const int STATE_REFILL_WAIT             = 6;
  const int STATE_TORQUING                = 7;

  // ------------------------  State Logic  -----------------------

  // Writes the logic to transition to the next state
  volatile int state_current;
  volatile int state_next;

//  if (e_stop){
//    state_current = STATE_ROOT;
//  }
//  else {
//    state_current = state_next;
//  }

  // ---------------------  State Transitions  --------------------

  // Fault trackers
  volatile bool faults;
  volatile bool solft_fault;
  volatile bool hard_fault;
  volatile bool fatal_fault;

  // True if the state machine passed throug the precharging process
  // False if there is no need to precharge again (hard & soft fault)

  volatile bool it_comes_from_HV;

  // Botton trackers
  volatile bool advance;
  volatile bool e_stop;

  //advance = 




  // -----------------------  State Outputs  ----------------------

  return state_current;
}

//------------------------------------------------
// 4. Tests for above functions
//------------------------------------------------
test(soft_fault_detection){
  // Tests if the soft fault it triggered by different potential values for vesc_readings
  // Artificial Reading     current motor_current voltage c_speed c_dist  power
  vesc_reading* vesc_val;
  vesc_val->current = 0;
  vesc_val->motor_current = 0;
  vesc_val->voltage = 0;
  vesc_val->c_speed = 2;
  vesc_val->c_dist = 0;
  vesc_val->power = 0;
 
  int fault_check = check_faults(vesc_val);
  assertEqual(fault_check,1);
}

test(hard_fault_detection){
  // Tests if the hard fault it triggered by different potential values for vesc_readings
  // Artificial Reading     current motor_current voltage c_speed c_dist  power
  vesc_reading* vesc_val;
  vesc_val->current = 0;
  vesc_val->motor_current = 0;
  vesc_val->voltage = 5;
  vesc_val->c_speed = 0;
  vesc_val->c_dist = 0;
  vesc_val->power = 0;
 
  int fault_check = check_faults(vesc_val);
  assertEqual(fault_check,2);
}


test(fatal_fault_detection){
  // Tests if the hard fault it triggered by different potential values for vesc_readings
  // Artificial Reading     current motor_current voltage c_speed c_dist  power
  vesc_reading* vesc_val;
  vesc_val->current = 10;
  vesc_val->motor_current = 0;
  vesc_val->voltage = 0;
  vesc_val->c_speed = 0;
  vesc_val->c_dist = 0;
  vesc_val->power = 0;
 
  int fault_check = check_faults(vesc_val);
  assertEqual(fault_check,10);
}



//---------------------------------------------------------------------------------------------
// VOID SETUP
//---------------------------------------------------------------------------------------------
void setup() {
//pinMode(hall_sensor, INPUT);
//pinMode(batt_volt_sensor, INPUT);
//pinMode(batt_current_sensor, INPUT);
//pinMode(wave_gauge, INPUT);
//pinMode(torque_sensor, INPUT);
//Serial.begin(9600);

//---------------------------------------------------------------------------------------------
// TEST SET-UP
//---------------------------------------------------------------------------------------------
Serial.begin(9600);
while(!Serial) {} // Portability for Leonardo/Micro

}


//---------------------------------------------------------------------------------------------
// VOID LOOP
//---------------------------------------------------------------------------------------------
void loop() {
  // put your main code here, to run repeatedly:

  //Read sensors
  //int batt_voltage_val = batt_voltage_read();
  //int batt_current_val = batt_current_sensor_read();
  //int torque_val = torque_sensor_read();
  //struct vesc_reading = vesc_read();

  //check_faults();
  //run_state_machine();

//---------------------------------------------------------------------------------------------
// RUN TESTS
//---------------------------------------------------------------------------------------------
  Test::run();

}
