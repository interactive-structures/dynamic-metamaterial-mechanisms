#include <Wire.h>
#include "Adafruit_MPR121.h"
#include <Adafruit_Sensor.h>
#define TCAADDR 0x70

const int NUM_CELLS = 1;

int sensors[NUM_CELLS][2] = {{0,1}};
const int ARRAY_SIZE = 2;

float angleLog[NUM_CELLS][ARRAY_SIZE] = {{70, 100}};

int k_left=0; int k_right=1; int b_left=2; int b_right=3; int supplyValveA=4; int exhaustValveA=5; int supplyValveB=6; int exhaustValveB=7; int targetReached=8;
double dic[NUM_CELLS][9];

double k0 = -1.72; double b0 = 281;
double k1 = 1.82; double b1 = -36.8;
//double k2 = -1.72; double b2 = 281;
//double k3 = 1.82; double b3 = -36.8;

double tolerance = 4.0;
int pulseLength[] = {25, 25};

long pulseCounter[] = {0, 0};
long cycleCounter[] = {0, 0};
long attemptCycles = 100;

double ks[][2] =      {{k0, k1}};
double bs[][2] =      {{b0, b1}};
double supplyValveAs[] = {3, 7};
double exhaustValveAs[] = {4, 8};
double supplyValveBs[] = {5, 9};
double exhaustValveBs[] = {6, 9};

Adafruit_MPR121 cap = Adafruit_MPR121();

int index = 0;

void TCA9548A(uint8_t bus) {
  Wire.beginTransmission(TCAADDR);  // TCA9548A address
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission(); 
}

double getAngle(int input, int sensorSide, int s){
  switch(sensorSide){
    case 0:
      return (input-dic[s][b_left])/dic[s][k_left];
    case 1:
      return (input-dic[s][b_right])/dic[s][k_right];
  }
}

void cycleIndex(){
  int targetsAchieved = 0;
  for (int i = 0; i<NUM_CELLS; i++){
    targetsAchieved += dic[i][targetReached];
  }
  if (targetsAchieved == NUM_CELLS){
    Serial.println("\n\nAll cells moving to next target\n\n");
    for (int i = 0; i<NUM_CELLS; i++){
      dic[i][targetReached] = 0;
    }
    if (index < ARRAY_SIZE - 1){
      index += 1;
    } else {
      index = 0;
    } 
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial){delay(10);} //from Adafruit MPR121 example

  for (int i = 0; i< NUM_CELLS; i++){
    dic[i][k_left] = ks[i][0];
    dic[i][k_right] = ks[i][1];
    dic[i][b_left] = bs[i][0];
    dic[i][b_right] = bs[i][1];
    dic[i][supplyValveA] = supplyValveAs[i];
    dic[i][exhaustValveA] = exhaustValveAs[i];
    dic[i][supplyValveB] = supplyValveBs[i];
    dic[i][exhaustValveB] = exhaustValveBs[i];
    dic[i][targetReached] = 0;
  }
        
  TCA9548A(0);
  if (!cap.begin(0x5A)) {
    Serial.println("MPR121 not found, check wiring?");
    while (1);
  }
  for (int i = 0; i< NUM_CELLS; i++){
    pinMode(dic[i][supplyValveA], OUTPUT);
    pinMode(dic[i][exhaustValveA], OUTPUT);
    pinMode(dic[i][supplyValveB], OUTPUT);
    pinMode(dic[i][exhaustValveB], OUTPUT);
  
    digitalWrite(dic[i][supplyValveA], LOW);
    digitalWrite(dic[i][exhaustValveA], HIGH);
    digitalWrite(dic[i][supplyValveB], LOW);
    digitalWrite(dic[i][exhaustValveB], HIGH);
    delay(500);
  }
}

void loop() {
  for (int i = 0; i<NUM_CELLS; i++){
    double currTarget = angleLog[i][index];
    //check current angle
    double currAngle;
    int left = cap.filteredData(sensors[i][0]);
    int right = cap.filteredData(sensors[i][1]);
    
    if (left <= right){
      currAngle = getAngle(left, 0, i);
    }
    else {;
      currAngle = getAngle(right, 1, i);
    }
    Serial.print("\tCell #"); Serial.print(i); Serial.print("-->");
    Serial.print("\tcurrTarget = "); Serial.print(currTarget);
    Serial.print("\tcurrAngle = "); Serial.print(currAngle); 
    double currDiff = currAngle - currTarget;
  //  Serial.print("\tcurrDiff = "); Serial.println(currDiff);
  
    //see if we have achieved target
    if (!dic[i][targetReached]){
      //if we have not achieved target, see if we are close to target
      if(abs(currDiff) < (tolerance * 2)){
        //if we are close to target, do two things: 1. shorten the length of each pulse and 2. lock the inactive bag
  //      Serial.print("moving to target slowly\t");
        if (cycleCounter[i] <= attemptCycles){
          cycleCounter[i]++;
          if (pulseCounter[i] % 2 == 0){
            pulseLength[i] = 15;
          } 
          else {
            pulseLength[i] = 100;
          }
          //see if we are in the region close to 90 deg
          if (abs(currTarget - 90) < 10){
            if (currDiff > 0){
    //          Serial.print(" moving right near 90 slowly ");
              digitalWrite(dic[i][supplyValveA], LOW);
              digitalWrite(dic[i][exhaustValveA], HIGH);
              digitalWrite(dic[i][supplyValveB], LOW);
              digitalWrite(dic[i][exhaustValveB], LOW);
            } else{
    //          Serial.print(" moving left near 90 slowly ");
              digitalWrite(dic[i][supplyValveA], LOW);
              digitalWrite(dic[i][exhaustValveA], LOW);
              digitalWrite(dic[i][supplyValveB], LOW);
              digitalWrite(dic[i][exhaustValveB], HIGH);
            }
          }
          else if (currDiff > 0){
    //        Serial.print(" moving right slowly ");
            digitalWrite(dic[i][supplyValveA], LOW);
            digitalWrite(dic[i][exhaustValveA], LOW);
            digitalWrite(dic[i][supplyValveB], HIGH);
            digitalWrite(dic[i][exhaustValveB], LOW);
          }
          else{
    //        Serial.print(" moving left slowly ");
            digitalWrite(dic[i][supplyValveA], HIGH);
            digitalWrite(dic[i][exhaustValveA], LOW);
            digitalWrite(dic[i][supplyValveB], LOW);
            digitalWrite(dic[i][exhaustValveB], LOW); 
          }
      } else{
        Serial.print("\n\t\tCell #"); Serial.print(i); Serial.println(" has failed to reach its target\n>");
        dic[i][targetReached] = 1;
        }
      }
      else {
        //if we are not close to the target do two things: 1. lengthen the time of each pulse and 2. vent inactive bag
        pulseLength[i] = 25;
        //see if we are close to 90 deg
        if (abs(currTarget - 90) < 10){
  //        Serial.print(" moving to 90 ");
          digitalWrite(dic[i][supplyValveA], HIGH);
          digitalWrite(dic[i][exhaustValveA], LOW);
          digitalWrite(dic[i][supplyValveB], HIGH);
          digitalWrite(dic[i][exhaustValveB], LOW);
        }
        else if (currDiff > 0){
  //        Serial.print(" moving right ");
          digitalWrite(dic[i][supplyValveA], LOW);
          digitalWrite(dic[i][exhaustValveA], HIGH);
          digitalWrite(dic[i][supplyValveB], HIGH);
          digitalWrite(dic[i][exhaustValveB], LOW);
        }
        else{
  //        Serial.print(" moving left ");
          digitalWrite(dic[i][supplyValveA], HIGH);
          digitalWrite(dic[i][exhaustValveA], LOW);
          digitalWrite(dic[i][supplyValveB], LOW);
          digitalWrite(dic[i][exhaustValveB], HIGH);  
        }
      }
      //maintain valve states for variable time (short or long)
      delay(pulseLength[i]);
      pulseCounter[i]++;
//      Serial.print("\tpulseLength = "); Serial.print(pulseLength[i]);
      
      //lock bagsa
      digitalWrite(dic[i][supplyValveA], LOW);
      digitalWrite(dic[i][exhaustValveA], LOW);
      digitalWrite(dic[i][supplyValveB], LOW);
      digitalWrite(dic[i][exhaustValveB], LOW);
      //delay(
      delay(400*(1/pulseLength[i])); 
    } else {
      Serial.print("\tCell #"); Serial.print(i);
      Serial.print(" has achieved its target ");
      Serial.print("\tcurrAngle = "); Serial.print(currAngle); 
      digitalWrite(dic[i][supplyValveA], LOW);
      digitalWrite(dic[i][exhaustValveA], LOW);
      digitalWrite(dic[i][supplyValveB], LOW);
      digitalWrite(dic[i][exhaustValveB], LOW);
    }
    if (abs(currDiff) <= tolerance){
      dic[i][targetReached] = 1;
      Serial.print("\n\t\tCell #"); Serial.print(i); Serial.println(" has reached its target\n>");
    }
  }
  Serial.println();
  cycleIndex();
}
