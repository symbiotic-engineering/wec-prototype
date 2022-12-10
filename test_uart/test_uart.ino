/*
Arduino UART communication with VESC. This software reads VESC telemetr data
and displays it on an OLED display. This code is under development for an
improved ppm remote control.
It is written by Sascha Ederer (roboshack.wordpress.com), based on the code
of jenkie (pedelecforum.de), Andreas Chaitidis (Andreas.Chaitidis@gmail.com)
and Benjamin Vedder (www.vedder.se).
Copyright (C) 2016
This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software Foundation,
Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
*/

// ---------------------------- Code Related to Reading Vesc Values over UART --

#include "config.h"
#include "datatypes.h"
#include "vescUart.h"
#include "HardwareSerial.h"
#include "vescUart.h"

    // For motor control, we will need to set current, ideally positon and will not set rpm
    // The provided library doesn't support commanding position, so we'll only set current

//Setup---------------------------------------------------------------------------------------------------------------------

void setup()
{
        Serial.begin(9600);
        Serial3.begin(9600);
	
	// show text
	Serial.println("System startup");

  // I think this waits for serial to start
  while (!Serial1) {;}

  // Define which port to use as UART
  VESCUART.setSerialPort(&Serial);
	
  delay(1000);
	
}
sea-lab@cornell.edu

void loop()
{
    if (vesc_get_values(VescMeasuredValues)) {
      
        // calculation of several values to be displayed later on
        voltage = VescMeasuredValues.v_in;
        current = VescMeasuredValues.current_in;
        motor_current = VescMeasuredValues.current_motor;
        power = current*voltage;
        c_speed = (VescMeasuredValues.rpm/38)*3.14159265359*0.000083*60;
        c_dist = (VescMeasuredValues.tachometer/38)*3.14159265359*0.000083;
        fault_code = VescMeasuredValues.fault_code;
        

    }
    else
    {
        
        // Error message when VESC is not connected or UART data
        // can not be read by the Arduino.
        // I reduced this message to a "blinking dot" as the code 
        // tends to get incomplete data packages on the Arduino Nano
        // while it runs errorfree on the Arduino Mega 
        

	Serial.println(".");

    }
    
  // display the calculated values      
	
	// show text battery voltage
	Serial.print(voltage);
  Serial.print("V");
  Serial.println();
        
  // show text drawn current
  Serial.print(motor_current);
  Serial.print("A");
  Serial.println();

	// show text watt
	Serial.print(power);
  Serial.print("W ");
  Serial.println();

	// show text distance
	Serial.print(c_dist);
  Serial.print("km ");
  Serial.println();
        
  // show text speed
  Serial.print(c_speed);
	Serial.print("km/h");
  Serial.println();

  // show text speed
  Serial.print("Fault Type:");
  Serial.print(fault_code);
  Serial.println();

}