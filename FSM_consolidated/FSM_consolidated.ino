#include <ezButton.h>
#include <VescUart.h>
#include <math.h>

/* Timer interrupt definitions */
#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0
#include "STM32TimerInterrupt.h"
#include "STM32_ISR_Timer.h"

// ---- Definitions -----

/** Initialize VescUart class */
VescUart UART;

enum State {ROOT, PRECHARGE, HIGH_VOLTAGE, SWITCH_WAIT, TORQUING, SWITCH_AUTO, POWER_DOWN};
enum State current_state = ROOT; 

volatile unsigned long previousMillis = 0;
volatile unsigned long currentMillis = 0;

int tach_norm = 0;

ezButton ADVANCE(6);
ezButton ESTOP(5);

volatile float theta = 0;

const float voltage_threshold = 35.2;
/* 
NO FAULTS: fault = 0
SOFT_FAULT: fault = 1
HARD_FAULT: fault = 2
FATAL_FAULT: fault = 3
*/
enum Fault {NO_FAULT, SOFT_FAULT, HARD_FAULT, FATAL_FAULT};
enum Fault fault = NO_FAULT;
volatile int batt_voltage = 0;
volatile int advance = 0;
volatile int estop = 0;

// We are creating a struct (short for structure) called vesc_reading, to hold all the values
// embedded in the VESC's output
struct vesc_reading{
    float current = 0.0;           //measured battery current
    float motor_current = 0.0;     //measured motor current
    float voltage = 0.0;           //measured battery voltage
    float c_speed = 0.0;           //measured rpm * Pi * wheel diameter [km] * 60 [minutes]
    float tach = 0.00;           //measured odometry tachometer [turns] * Pi * wheel diameter [km] 
    double power = 0.0;              //calculated power
    int error = mc_fault_code(FAULT_CODE_NONE);
};


//---------------------------------------------------------------------------------------------
// VARIABLES
//---------------------------------------------------------------------------------------------

//ASSIGNING VARIABLES TO EACH OF THE PIN CONNECTIONS
//const int hallsensorPin; //= pin number of the hall_sensor
const int battvoltsensorPin = A4; //= pin number of the battery voltage sensor
const int battcurrentsensorPin = A5; //= pin number of the battery current sensor
const int wavegaugePin = A3; //= pin number of the wave gauge
const int torquesensorPin = A1; //=pin number of the torque sensor
#define HIGH_SIDE_RELAY_PIN   23
#define PRE_LOW_DIS_RELAY_PIN 25
#define VESC_POWER_PIN 24


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
        
    vesc_val.c_speed = (UART.data.rpm/21)*(3.14159265359/60); /*(UART.data.rpm)*(3.14159265359/60);*/
    vesc_val.tach = UART.data.tachometer;
        
    vesc_val.power = vesc_val.current*vesc_val.voltage;
    vesc_val.error = UART.data.error;

    // Serial.print("Vesc Current = ");
    // Serial.println(vesc_val.current);
    // Serial.print("Vesc Voltage = ");
    // Serial.println(vesc_val.voltage);
    // Serial.print("Motor Current = ");
    // Serial.println(vesc_val.motor_current);
    // Serial.print("Speed = ");
    // Serial.println(vesc_val.c_speed);
    // Serial.print("tach = ");
    // Serial.println(vesc_val.tach);
    // Serial.print("Power = ");
    // Serial.println(vesc_val.power);
    // Serial.print("error = ");
    // Serial.println(vesc_val.error);
  }
  else
  {
    //Serial.println("Failed to get data!");
  }

  return vesc_val;
}

struct vesc_reading current_vesc;


// -------------------------------- battery voltage -----------
// reference: https://github.com/BasOnTech/Arduino-Beginners-EN/blob/master/E17-voltage-sensor/voltage-sensor.ino
float batt_voltage_read(){
  // Returns: voltage value, type float
  float analog_reading = analogRead(battvoltsensorPin); //the 1024 is included because that is the number of int in 10 bits
  float V_adc = analog_reading/1024*3.3; //voltage level of microcontroller pin
  float V_sensor = V_adc * 37/10; //the 37/10 is included because a voltage divider with R1= 27 kohms and R2=10 kohms is used to bring down sensor voltage of 12V to 3.3V
  float batt_voltage_val = V_sensor * 4.5568; //https://cdn.sparkfun.com/assets/c/a/a/4/6/Voltage_to_Voltage_45a.png
  // Serial.print("Battery Voltage = ");
  // Serial.println(batt_voltage_val);
  
  return batt_voltage_val;

}

// -------------------------------- battery current -----------
float batt_current_sensor_read(){
  // Returns: a current value, type float
  float analog_reading = analogRead(battcurrentsensorPin); //the 1024 is included because that is the number of int in 10 bits
  float V_adc = analog_reading/1024*3.3; //voltage level of microcontroller pin
  float batt_current_val = V_adc * 13.67; //https://cdn.sparkfun.com/assets/8/a/9/4/b/Current_to_Voltage_45a.png

  // Serial.print("Battery Current= ");
  // Serial.println(batt_current_val);
  // Serial.println(V_adc);
  // Serial.println(analog_reading);

  return batt_current_val;
}

// -------------------------------- wave gauge -----------

float wave_gauge_read(){
  //Returns: wave height value, type float

  float k = 25.7069; //conversion factor found experimentally
  float analog_reading = analogRead(wavegaugePin); //the 1024 is included because that is the number of int in 10 bits
  float V_adc = analog_reading / 1024 * 3.3; //voltage level of microcontroller pin
  float V_sensor = V_adc * 15/5; //the 15/5 is included because a voltage divider with R1= 10 kohms and R2=5 kohms is used to bring down sensor voltage of 10V to 3.3V
  float wave_gauge_val = k*V_sensor+31.619487; //y-intercept of 31.619487

  // Serial.print("Wave gauge = ");
  // Serial.println(wave_gauge_val);
  

  return wave_gauge_val;
}


// -------------------------------- torque sensor -----------

float torque_sensor_read(){
  // Returns: torque value, type float
 
  float k = 1.0361; //conversion factor found experimentally, note that there is also a y intercept in the graph
  float analog_reading = analogRead(torquesensorPin); //the 1024 is included because that is the number of int in 10 bits
  float V_adc = analog_reading / 1024 * 3.3; //voltage level of microcontroller pin
  float V_sensor_normalized = V_adc * 15/10; //the 15/10 is included because a voltage divider with R1= 5 kohms and R2=10 kohms is used to bring down sensor voltage of 5V to 3.3V
  float V_sensor = 2 * (V_sensor_normalized - 2.595); //the sensor output is normalized from (-5,5) -> (0,5) using a resistor network
  //https://oshgarage.com/reading-negative-voltages-with-an-adc-using-passives/
  float torque_val = k*V_sensor+0.1745; //y-intercept of 0.1745

  // Serial.print("Torque = ");
  // Serial.println(torque_val);

  return torque_val;
}

// Init STM32 timer TIM1
STM32Timer ITimer0(TIM1);
#define TIMER0_INTERVAL_MS 10

void setup() {
  // put your setup code here, to run once:

  //Initialize serial monitor
  //Note: make sure this matches the baud rate of your serial monitor
  Serial.begin(9600);

  //Intialize Timer Interrupt
  //Timer is set to interrupt every 10ms creating a polling frequency of 100Hz
  // TIMER0_INTERVAL_MS is multiplied by 1000 because the attachInterruptInterval function uses microseconds not miliseconds
  if (ITimer0.attachInterruptInterval(TIMER0_INTERVAL_MS * 1000, timer_ISR_state_machine))
    Serial.println("Starting  ITimer0 OK, millis() = " + String(millis()));
  else
    Serial.println("Can't set ITimer0. Select another freq. or timer");

  ADVANCE.setDebounceTime(50);
  

  // Set up STM output pins
  pinMode(HIGH_SIDE_RELAY_PIN, OUTPUT);
  pinMode(PRE_LOW_DIS_RELAY_PIN, OUTPUT);

  pinMode(VESC_POWER_PIN, OUTPUT);

  pinMode(battvoltsensorPin, INPUT);
  pinMode(battcurrentsensorPin, INPUT);
  pinMode(wavegaugePin, INPUT);
  pinMode(torquesensorPin, INPUT);


  /** Setup UART port (Serial3 on STM32 Feather) */
  Serial3.begin(115200);
  while (!Serial) {;}
  /** Define which ports to use as UART */
  UART.setSerialPort(&Serial3);
}

//----------------------------
// Interrupt Handler
//----------------------------
void timer_ISR_state_machine(){
  //Read from VESC
  current_vesc = vesc_read();
  //Set appropriate flags for state transisitons
  flag_set(current_vesc);
  //Calculate motor angular position from tachometer reading  
  theta = 2 * 3.14159 * (current_vesc.tach/126);
  // Serial.print("Angle = ");
  // Serial.println(degrees(theta));
  switch(current_state){
    case ROOT: // (ROOT->PRECHARGE) advance && !faults
      batt_voltage = 0;
      digitalWrite(HIGH_SIDE_RELAY_PIN, LOW);
      digitalWrite(PRE_LOW_DIS_RELAY_PIN, LOW);
      digitalWrite(VESC_POWER_PIN, LOW);
      if(advance == 1 /*&& fault == NO_FAULT*/){
        current_state = PRECHARGE;
        Serial.println(current_state);
      }
    
      break;
    case PRECHARGE: // (PRECHARGE->HIGH_VOLTAGE) !faults && bat_voltage_is_high_enough
      digitalWrite(HIGH_SIDE_RELAY_PIN, LOW);
      digitalWrite(PRE_LOW_DIS_RELAY_PIN, HIGH);
      if(batt_voltage == 1){
        current_state = HIGH_VOLTAGE;
        Serial.println(current_state);
      }
      else if(fault == FATAL_FAULT){
        current_state = ROOT;
        Serial.println(current_state);
      }
      break;
    case HIGH_VOLTAGE: // (HIGH_VOLTAGE->SWITCH_WAIT) advance && !faults
      digitalWrite(HIGH_SIDE_RELAY_PIN, HIGH);
      digitalWrite(PRE_LOW_DIS_RELAY_PIN, HIGH);
      if(advance == 1 && fault == NO_FAULT){
        current_state = SWITCH_WAIT;
        Serial.println(current_state);
      }
      else if(fault == FATAL_FAULT){
        current_state = ROOT;
        Serial.println(current_state);
      }
      break;
    case SWITCH_WAIT: // (SWITCH_WAIT->TORQUING) advance && !faults
      digitalWrite(VESC_POWER_PIN, HIGH);
      //UART.setCurrent(0);
      if(advance == 1 && fault == NO_FAULT){
        current_state = TORQUING;
        Serial.println(current_state);
      }
      else if(fault == FATAL_FAULT){
        current_state = ROOT;
        Serial.println(current_state);
      }
      break;
    case TORQUING: // (TORQUING->TORQUING) !faults
      if(fault == FATAL_FAULT){ // (TORQUING->ROOT) fatal_fault
        current_state = ROOT;
        Serial.println(current_state);
      }
      else if(fault == HARD_FAULT){ // (TORQUING->HIGH_VOLTAGE) hard_fault
        current_state = HIGH_VOLTAGE;
        Serial.println(current_state);
      }
      else if(fault == SOFT_FAULT){ // (TORQUING->SWITCH_AUTO) soft_fault
        current_state = SWITCH_AUTO;
        Serial.println(current_state);
      }
      else if(advance == 1 && fault == NO_FAULT){ // (POWER_DOWN) !faults && advance
        current_state = POWER_DOWN;
        previousMillis = millis();
        Serial.println(current_state);
      }   
      else{
        UART.setCurrent(calcVoltage(1, current_vesc.c_speed, 2, theta, 3));
        //calcVoltage(1, current_vesc.c_speed, 2, theta, 3);
      }
      break;
    case SWITCH_AUTO: // (SWITCH_AUTO->TORQUING)!faults
      if(fault == NO_FAULT){
        current_state = TORQUING;
        Serial.println(current_state);
      }
      else if(fault == FATAL_FAULT){
        current_state = ROOT;
        Serial.println(current_state);
      }
      break;
    case POWER_DOWN: // (SWITCH_AUTO->TORQUING)!faults
      UART.setCurrent(0);
      currentMillis = millis();
      if(currentMillis - previousMillis > 3000){
        current_state = ROOT;
        Serial.println(current_state);
      }
      else if(fault == FATAL_FAULT){
        current_state = ROOT;
        Serial.println(current_state);
      }
      break;
  }
  advance = 0;
  estop = 0;
}

//---------------------------------
// Sets flags for state transitions
//---------------------------------
void flag_set(vesc_reading temp_vesc){
  ADVANCE.loop(); // polls advance button for
  ESTOP.loop(); // polls estop button

  // Sets advance to 1 if pressed
  if (ADVANCE.isPressed())
    {
    if (advance == 0){
      advance = 1;
      }
    else {advance = 0;}
    }   
  // Sets estop to 1 if pressed
  if (ESTOP.isPressed()){
    if (estop == 0){
      estop = 1;
      }
    else {
      estop = 0;
    }
  }
  // Sets batt_voltage to 1 if the voltage sensor reading exceeds voltage_threshold
  if(batt_voltage_read() >= voltage_threshold){
    batt_voltage = 1;
    // Serial.println(batt_voltage_read());
  }

  // Set faults
  if((temp_vesc.error == FAULT_CODE_OVER_VOLTAGE) || (temp_vesc.error == FAULT_CODE_UNDER_VOLTAGE) || (temp_vesc.error == FAULT_CODE_DRV) || (estop == 1)){
    // Set fatal fault
    fault = FATAL_FAULT; 
  } else if ((temp_vesc.error == FAULT_CODE_ABS_OVER_CURRENT )|| (temp_vesc.error == FAULT_CODE_OVER_TEMP_FET)){
    // Set hard fault
    fault = HARD_FAULT; 
  } else {
    // TODO: Add soft fault generation based on angle
    // No faults
    fault = NO_FAULT;
  }

}

void loop() {
  // put your main code here, to run repeatedly:
  // current_vesc = vesc_read();
  // flag_set(current_vesc);

  float batt_voltage_val = batt_voltage_read();
  float batt_current_val = batt_current_sensor_read();
  float wave_gauge_val = wave_gauge_read();
  float torque_val = torque_sensor_read();  

}
