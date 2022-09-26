#include <Wire.h>
#include "Adafruit_MPR121.h"
#include <Adafruit_Sensor.h>
#define TCAADDR 0x70

const int NUM_CELLS = 6;

int sensors[NUM_CELLS][2] = {{0,1}, {2, 3}, {4, 5}, {6, 7}, {8, 9}, {10, 11}};
const int ARRAY_SIZE = 2;

float angleLog[NUM_CELLS][ARRAY_SIZE] = {{70, 100}, {70, 100}, {70, 100}, {70, 100}, {70, 100}, {70, 100}};

int k_left=0; int k_right=1; int b_left=2; int b_right=3; int supplyValveA=4; int exhaustValveA=5; int supplyValveB=6; int exhaustValveB=7; int targetReached=8;
double dic[NUM_CELLS][9];

//cell 1 (confirm that this is cell #7)
double k0 = -1.72; double b0 = 281;
double k1 = 1.82; double b1 = -36.8;
//cell 2 (uncalibrated, using generic values)
double k2 = -2.0; double b2 = 310;
double k3 = 2; double b3 = -52;
//cell 3 (uncalibrated, using generic values)
double k4 = -2.0; double b4 = 310;
double k5 = 2; double b5 = -52;
//cell 4 (uncalibrated, using generic values)
double k6 = -2.0; double b6 = 310;
double k7 = 2; double b7 = -52;
//cell 5 (uncalibrated, using generic values)
double k8 = -2.0; double b8 = 310;
double k9 = 2; double b9 = -52;
//cell 6 (cell "C")
double k10 = -2.42; double b10 = 353;
double k11 = 2.25; double b11 = -68.9;

double tolerance = 4.0;
int pulseLength[] = {25, 25, 25, 25, 25, 25};

long pulseCounter[] = {0, 0, 0, 0, 0, 0};
long cycleCounter[] = {0, 0, 0, 0, 0, 0};
long attemptCycles = 100;

double ks[][2] =      {{k0, k1}, {k2, k3}, {k4, k5}, {k6, k7}, {k8, k9}, {k10, k11}};
double bs[][2] =      {{b0, b1}, {b2, b3}, {b4, b5}, {b6, b7}, {b8, b9}, {b10, b11}};
double supplyValveAs[] = {2, 6, 10, 14, 18, 24};
double exhaustValveAs[] = {3, 7, 11, 15, 19, 25};
double supplyValveBs[] = {4, 8, 12, 16, 22, 26};
double exhaustValveBs[] = {5, 9, 13, 17, 23, 27};

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
   
    double currDiff = currAngle - currTarget;
  //  Serial.print("\tcurrDiff = "); Serial.println(currDiff);
  
    //see if we have achieved target
    if (!dic[i][targetReached]){
      Serial.print("\t#"); Serial.print(i); Serial.print("->");
      Serial.print("\tTarget= "); Serial.print(currTarget);
      Serial.print("\tAngle= "); Serial.print(currAngle); 
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
        cycleCounter[i] = 0;
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
      Serial.print("\t\tCell #"); Serial.print(i);
      Serial.print(" target achieved\t");
//      Serial.print("\tcurrAngle = "); Serial.print(currAngle); 
      digitalWrite(dic[i][supplyValveA], LOW);
      digitalWrite(dic[i][exhaustValveA], LOW);
      digitalWrite(dic[i][supplyValveB], LOW);
      digitalWrite(dic[i][exhaustValveB], LOW);
    }
    if (abs(currDiff) <= tolerance){
      dic[i][targetReached] = 1;
//      Serial.print("\n\t\tCell #"); Serial.print(i); Serial.println(" has reached its target\n>");
    }
  }
  Serial.println();
  cycleIndex();
}