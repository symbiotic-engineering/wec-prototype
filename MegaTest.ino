#include <VescUart.h>

VescUart UART;

float current = 2.10;

String message;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  while (!Serial1) {;}

  UART.setSerialPort(&Serial1);

}

void loop(){

  if(Serial.available() > 0) {
   message = Serial.readStringUntil('\n');
  }

  if(UART.getVescValues() && message == "Osc") {
    UART.setCurrent(current);
  }

  //Serial.println(UART.data.avgInputCurrent);
  //delay(100);

    //Serial.println("Hello World!");
    //UART.printVescValues();
    //UART.data.avgMotorCurrent
  //}
//   else{
//     Serial.println("Not reading");
//   }
// delay(1000);
}
