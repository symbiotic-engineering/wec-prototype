#include <bsp_sd.h>
#include <ffconf.h>
#include <ffconf_default_32020.h>
#include <ffconf_default_68300.h>
#include <Sd2Card.h>
#include <SdFatFs.h>
#include <STM32SD.h> // git clone "https://github.com/stm32duino/STM32SD.git"

/*
  SD card datalogger
 This example shows how to log data from three analog sensors
 to an SD card using the SD library.
 The circuit:
 * analog sensors on analog ins A0, A1, and A2
 * SD card
 */



// If SD card slot has no detect pin then define it as SD_DETECT_NONE
// to ignore it. One other option is to call 'SD.begin()' without parameter.
#ifndef SD_DETECT_PIN
#define SD_DETECT_PIN SD_DETECT_NONE
#endif

uint32_t A[] = { A0, A1, A2};

File dataFile;

void setup()
{
  
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  while (!SD.begin(SD_DETECT_PIN))
  {
    delay(10);
  }
  delay(100);
  Serial.println("card initialized.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  // if the file is available, seek to last position
  if (dataFile) {
    dataFile.seek(dataFile.size());
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}

void log_data(String current_state, bool fatal_fault, bool hard_fault, bool soft_fault, float reading_a){
  double t = millis()/1000;
  String dataString = String(t);
  dataString += ", " + current_state;
  String fault = "none";
  if (soft_fault){fault = "Soft";}
  if (hard_fault){fault = "Hard";}
  if (fatal_fault){fault = "Fatal";}
  dataString += ", " + fault;
  dataString += ", " + String(reading_a);
  
  
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.flush(); // use flush to ensure the data written
    // print to the serial port too:
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error on datalog.txt file handle");
  }
  delay(100);
}

void loop()
{
  
}
