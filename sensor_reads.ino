// Power System Sensor Reading and State Machine Module
// 
// This module is split into two sections: the function d

// ---- Importing Libraries -----

#include "vesc_uart.h"


// ---- Definitions -----



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
const float vCC_voltmeter = 13.6; 




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
// TO DO 
// Implement UART vesc readings
// Reference: https://github.com/R0b0shack/VESC-UART-Arduino/blob/master/VESC_UART_Nano.ino

void vesc_read(){
  // Returns: a parsed, UART reading, type vesc_reading (the struct we defined at the beginning)

  reading_hall = analogRead(hallsensorPin)
  struct vesc_reading vesc_val;

  if (vesc_get_values(VescMeasuredValues)) {
      
        // calculation of several values to be displayed later on
        vesc_val.current = VescMeasuredValues.current_in;
        vesc_val.voltage = VescMeasuredValues.v_in;
        
        vesc_val.motor_current = VescMeasuredValues.current_motor;
        
        vesc_val.c_speed = (VescMeasuredValues.rpm/38)*3.14159265359*0.000083*60;
        vesc_val.c_dist = (VescMeasuredValues.tachometer/38)*3.14159265359*0.000083;
        
        vesc_val.power = current*voltage;

  }
  //TODO: complete code to read the values above
  

  return vesc_val
}

// -------------------------------- battery voltage -----------
//TO document
// reference: https://github.com/BasOnTech/Arduino-Beginners-EN/blob/master/E17-voltage-sensor/voltage-sensor.ino
void batt_voltage_read(){
  voltage_val = analogRead(battvoltsensorPin);
  vOut = (reading_voltage/1024)*vCC; //read the current sensor value (0-1023) 
  //We might need a voltage divider if we want max voltage to be 3.3 int he case of voltage measurements
  vIn = vOut*factor:
  Serial.print("Voltage = ");
  Serial.print(vIn);
  Serial.println("V");
  
  return batt_voltage_val

}

// -------------------------------- battery current -----------
void batt_current_sensor_read(){
  // Returns: a current value, type int
  reading_current = analogRead(battcurrentsensorPin);
  bat_current_val = (reading_current/1024)*3.3; //https://cdn.sparkfun.com/assets/8/a/9/4/b/Current_to_Voltage_45a.png
  
  Serial.print("Source current= ");
  Serial.print(current);

  return batt_current_val
}

// -------------------------------- wave gauge -----------

//TODO 
void wave_gauge_read(){
  //Returns: wave height value, type int
  int wave_gauge_val = analogRead(wavegaugePin);

  return wave_gauge_val;
}


// -------------------------------- torque sensor -----------

void torque_sensor_read(){
  // Torque sensor conversion ratio is k

  // Returns: torque value, type int
  // Note that 5 is a dummy value we have to CHANGE
  int k = 1.0361; //conversion factor found experimentally, note that there is also a y intercept in the graph
  int Vin_torque_sensor = 5;
  int V_modified = Vin_torque_sensor*(10/15) //the 10/15 is included because a voltage divider with R1= 5 kohms and R2=10 kohms is used to bring down Vcc of 5V to 3.3V
  int torque_sensor_reading = analogRead(torquesensorPin);
  int torque_val = k*(torque_sensor_reading/1024)*V_modified+0.1745; 
  //the 1024 is included because that is the number of int in 10 bits
  //the 0.1745 is included because that's where the torque sensor was zeroed

  return torque_val;
}


//------------------------------------------------
// 2. Check Faults
//------------------------------------------------

char check_faults (struct vesc_reading* vesc_val){
  // Returns: the kind of fault (soft_fault / hard_fault / fatal_fault) if fault, otherwise default is "none"
  // Parameter vesc_val: the output value of the vesc, type "struct vesc_reading" (we defined the vesc_reading structure in line 9)

      // Note that we add the asterisk (*) because the parameter is in reality the pointer to the memory location of the 
      //      structure's first value. C is very low level and you have to pass on where things are located in memory 

  
  char fault_type = "none";
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
  volatile state_current;
  volatile state_next;

  if (e_stop){
    state_current = STATE_ROOT;
  }
  else {
    state_current = state_next;
  }

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

  return state_current
}



//---------------------------------------------------------------------------------------------
// VOID SETUP
//---------------------------------------------------------------------------------------------
void setup() {
pinMode(hall_sensor, INPUT);
pinMode(batt_volt_sensor, INPUT);
pinMode(batt_current_sensor, INPUT);
pinMode(wave_gauge, INPUT);
pinMode(torque_sensor, INPUT);
Serial.begin(9600);
}


//---------------------------------------------------------------------------------------------
// VOID LOOP
//---------------------------------------------------------------------------------------------
void loop() {
  // put your main code here, to run repeatedly:

  //Read sensors
  int batt_voltage_val = batt_voltage_read();
  int batt_current_val = batt_current_sensor_read();
  int torque_val = torque_sensor_read();
  struct vesc_reading = vesc_read();

  check_faults();
  run_state_machine();

  
}
