//#include <SWOStream.h>
//SWOStream s(168000000, SWO_Async, 0, false); //swoEnable = false
#define LED_PIN 13

void setup() {
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
  // while (!Serial) {
  //   ; //wait for serial port to connect
  // }
}

void loop() {
  Serial.println("Serial is working!");
  digitalWrite(LED_PIN, HIGH);
  //s.print("Serial is alive!");
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);
}
