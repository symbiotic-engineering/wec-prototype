// Power System Sensor Reading and State Machine Module
// 
// This module is split into two sections: the function d

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

// TO DO 
// Implement UART vesc readings
// Reference: https://github.com/R0b0shack/VESC-UART-Arduino/blob/master/VESC_UART_Nano.ino

void vesc_read(){
  // Returns: a parsed, UART reading, type vesc_reading (the struct we defined at the beginning)

  reading_hall = analogRead(hallsensorPin)
  vesc_reading vesc_val;

  vesc_val->current = 0.0;           //measured battery current
  vesc_val->motor_current = 0.0;     //measured motor current
  vesc_val->voltage = 0.0;           //measured battery voltage
  vesc_val->c_speed = 0.0;           //measured rpm * Pi * wheel diameter [km] * 60 [minutes]
  vesc_val->c_dist = 0.00;           //measured odometry tachometer [turns] * Pi * wheel diameter [km] 
  vesc_val->power = 0.0;              //calculated power

  //TODO: complete code to read the values above
  

  return vesc_val
}

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
  
  return voltage_val

}
void batt_current_sensor_read(){
  // Returns: a current value, type int
  reading_current = analogRead(battcurrentsensorPin);
  bat_current_val = (reading_current/1024)*3.3; //https://cdn.sparkfun.com/assets/8/a/9/4/b/Current_to_Voltage_45a.png
  
  Serial.print("Source current= ");
  Serial.print(current);

  return bat_current_val
}
//TODO 
void wave_gauge_read(){
  //Returns: wave height value, type int
  int wave_gauge_val = analogRead(wavegaugePin);

  return wave_gauge_val;
}

//TODO 
//I think it needs to be calibrated manually because there is no data sheet for it. For this you use a known inertia in the output, accelerate at a know veloctiy and use Newton's II law torque = inertia*acceleration and measured the voltage output.
//Measures torque from 0.5-150Nm and has an output signal of 0-20mA (not sure what capacity the one being used has)
//https://www.ato.com/micro-reaction-torque-sensor-0d5-nm-to-150-nm
//looks like it is connected to two pins

void torque_sensor_read(){
  // Torque sensor conversion ratio is k

  // Returns: torque value, type int
  // Note that 5 is a dummy value we have to CHANGE
  int k = 5;

  int torque_sensor_reading = analogRead(torquesensorPin);
  //do conversion  here
  int torque val = k*torque_sensor_read;

  return torque_val;
}


char run_state_machine(int wave_gauge_val, int torque_val, struct vesc_reading vesc_val){
  // Returns: the state machine's current state
  // Paramenter wave_gauge_val: the height of the wave, type int
  // Paramenter torque_val: the torque value read by the torque sensor, type int
  // Paramenter vesc_val: the height of the wave, type int

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


}

//---------------------------------------------------------------------------------------------
// Finite State Machine 
//---------------------------------------------------------------------------------------------
// Diagram available at https://tinyurl.com/7mwzab2m

void loop() {
  // put your main code here, to run repeatedly:

  
}
