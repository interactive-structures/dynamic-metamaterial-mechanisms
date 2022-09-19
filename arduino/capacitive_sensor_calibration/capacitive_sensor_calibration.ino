/*This file contains two versions of the same program:
 * In the top version, the data is not buffered.
 * In the bottom version, the data is buffered.
 * To switch between version, comment out everything in the inactive version, including libraries.
 * 
 * The unbuffered version is useful for seeing what is actually happening with the sensor, while
 * the buffered version provides a cleaner datastream for an actual control program. 
 */

#include <Wire.h>
#include "Adafruit_MPR121.h"
#include <Adafruit_Sensor.h>
#define TCAADDR 0x70

const uint8_t NUM_SENSORS = 2;

const int SENSOR_DELAY = 0;

Adafruit_MPR121 cap = Adafruit_MPR121();

void TCA9548A(uint8_t bus) {
  Wire.beginTransmission(TCAADDR);  // TCA9548A address
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}

void setup() {
Serial.begin(115200);
  Wire.begin();
  //Serial.println("Data Plotter for Adafruit MPR121 Capacitive Touch Board"); 
  Serial.println();
  Serial.println("Cap Sensor:      ");
  TCA9548A(0);
  // Default I2C addr is 0x5A
  if (!cap.begin(0x5A)) {
    Serial.println("MPR121 not found, check wiring?");
    while (1);
  }
}

void loop() {  
  TCA9548A(0);
  for (uint8_t s=0; s<NUM_SENSORS; s++) {
    if (s%2==0){Serial.print("\tleft = ");}
    else{Serial.print("\tright = ");}
    Serial.print(cap.filteredData(s));  Serial.print(", ");
  }
  Serial.println();
  delay(SENSOR_DELAY);
}

//----------------------------the buffered version of the program is below:

//#include <Wire.h>
//#include "Adafruit_MPR121.h"
//#include <Adafruit_Sensor.h>
//#define TCAADDR 0x70
//
//const uint8_t NUM_SENSORS = 2;
//
//Adafruit_MPR121 cap = Adafruit_MPR121();
//
//const uint16_t BUFFER_SIZE = 50;
//uint16_t buffer[NUM_SENSORS][BUFFER_SIZE];
//static int buffer_next = 0;
//static int buffer_cur = 0;
//static uint32_t buffer_sum[NUM_SENSORS];
//
//
//void TCA9548A(uint8_t bus) {
//  Wire.beginTransmission(TCAADDR);  // TCA9548A address
//  Wire.write(1 << bus);          // send byte to select bus
//  Wire.endTransmission();
//}
//
//void setup() {
//Serial.begin(115200);
//  Wire.begin();
//  //Serial.println("Data Plotter for Adafruit MPR121 Capacitive Touch Board"); 
//  Serial.println();
//  Serial.println("Cap Sensor:      ");
//  TCA9548A(0);
//  // Default I2C addr is 0x5A
//  if (!cap.begin(0x5A)) {
//    Serial.println("MPR121 not found, check wiring?");
//    while (1);
//  }
//  cap.setThresholds(2, 1);
//}
//
//void loop() {
//  uint16_t val, old_val;
//  
//  TCA9548A(0);
//  
//  for (uint8_t s=0; s<NUM_SENSORS; s++) {
//      old_val = buffer[s][buffer_next];
//      val = cap.filteredData(s);
//      buffer[s][buffer_next] = val;
//      buffer_sum[s] -= old_val;
//      buffer_sum[s] += val;
//  } 
//  buffer_cur = buffer_next;
//  buffer_next++;
//  if (buffer_next >= BUFFER_SIZE) buffer_next = 0;
//  for (uint8_t s=0; s<NUM_SENSORS; s++) {
//    if (s%2==0){Serial.print("\tleft = ");}
//    else{Serial.print("\tright = ");}
//    Serial.print(buffer_sum[s] / BUFFER_SIZE);  Serial.print(", ");
//  }
//  Serial.println();
//}
