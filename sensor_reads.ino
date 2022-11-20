-----VARIABLES-----
//ASSIGNING VARIABLES TO EACH OF THE PIN CONNECTIONS
const int hallsensorPin; //= pin number of the hall_sensor
const int battvoltsensorPin; //= pin number of the battery voltage sensor
const int battcurrentsensorPin; //= pin number of the battery current sensor
const int wavegaugePin; //= pin number of the wave gauge
const int torquesensorPin; //=pin number of the torque sensor
//ASSIGNING VARIABLES FOR THE READINGS OF EACH OF THE PINS
int reading_hall_sensor;

float reading_voltage; //value on pin
float vIn; // measured voltage 
float vOut;
const float factor = 4.16; //Vin/Vout https://cdn.sparkfun.com/assets/c/a/a/4/6/Voltage_to_Voltage_45a.png reduction factor of the Voltage Sensor Shield
const float vCC = 13.6; //input voltage for sensor


float reading_current;
float current;

int reading_wave_gauge;
int reading_torque_sensor;

void setup() {
pinMode(hall_sensor, INPUT);
pinMode(batt_volt_sensor, INPUT);
pinMode(batt_current_sensor, INPUT);
pinMode(wave_gauge, INPUT);
pinMode(torque_sensor, INPUT);
Serial.begin(9600);
}

void hall_sensor_read(){
//NEEDS TO BE DONE
}
void batt_voltage_read(){
  reading_voltage = analogRead(battvoltsensorPin);
  vOut = (reading_voltage/1024)*vCC; //read the current sensor value (0-1023)
  vIn = vOut*factor:
  Serial.print("Voltage = ");
  Serial.print(vIn);
  Serial.println("V");
  // reference: https://github.com/BasOnTech/Arduino-Beginners-EN/blob/master/E17-voltage-sensor/voltage-sensor.ino 
}
void batt_current_sensor_read(){
  reading_current = analogRead(battcurrentsensorPin);
  current = (reading_current/1024)*Vcc; //13.6 is the reference voltage for analog read in this sensor //https://cdn.sparkfun.com/assets/8/a/9/4/b/Current_to_Voltage_45a.png
  Serial.print("Source current= ");
  Serial.print(current);
}
void wave_gauge_read(){
  //NEEDS TO BE DONE
}

void torque_sensore_read(){
  //NEEDS TO BE DONE
}
void loop() {
  // put your main code here, to run repeatedly:

}
