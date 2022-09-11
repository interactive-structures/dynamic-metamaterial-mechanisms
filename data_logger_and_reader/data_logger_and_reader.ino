#include <Wire.h> //this is used to communication with the multiplexer
#include "Adafruit_MPR121.h" // this is the capacitive sensor board library
#define TCAADDR 0x70 //this is the multiplexer
#include <Adafruit_Sensor.h> //the is used for the capacitive sensor
#include <PID_v1.h> //this is the standard PID library

const byte BUTTON_PIN= 19;

const uint8_t NUM_CELLS = 2;
const int ARRAY_SIZE = 4;
int sensorPairs[6][2] = {{0,1}, {2,3}, {4,5}, {6,7}, {8,9}, {10,11}};

int CAP=0; int a_left = 1; int a_right = 2; int b_left=3; int b_right=4; int c_left=5; int c_right=6; int currDiff=7; int supplyValveA=8; int exhaustValveA=9; int supplyValveB=10; int exhaustValveB=11; int currAngle = 12; int currTarget = 13; 
double dic[NUM_CELLS][17];

//sensor constants from spreadsheet.
//With the sensor oriented like this: |∩|, list the constants for the left sensor, then the right.
//constants will be paired (k0, k1), (k2, k3), etc.
float a0 = -0.0311; double b0 = 4.84; double c0 = -49;
float a1 = -0.0268; double b1 = 4.48; double c1 = -36.2;
float a2 = -0.0267; double b2 = 4.28; double c2 = -33.4;
float a3 = -0.0276; double b3 = 4.44; double c3 = -36.2;

//arrays of sensor constants and valve pins (matched by index).
//pair the left and right sensor of each cell (when sensor is oriented like this : |∩|)
double as[][2] =      {{a0, a1}};
double bs[][2] =      {{b0, b1}};
double cs[][2] =      {{c0, c1}};
double supplyValveAs[] = {2, 6};
double exhaustValveAs[] = {3, 7};
double supplyValveBs[] = {4, 8};
double exhaustValveBs[] = {5, 9};

enum states {
  NOT_REACHED,
  ALL_REACHED
};
enum states state;

volatile byte logState = 0;

double Output[NUM_CELLS];
double targetDiff;

PID PIDs[] = {
   PID(&dic[0][currDiff], &Output[0], &targetDiff, .30, .8, 0, P_ON_M, DIRECT)//0.5,1
//   PID(&dic[1][currDiff], &Output[1], &targetDiff, 2, 10, 0, DIRECT), //0.5,1
//   PID(&dic[2][currDiff], &Output[2], &targetDiff, .5, 1, 0, DIRECT),//0.5,1
//   PID(&dic[3][currDiff], &Output[3], &targetDiff, .5, 1, 0, DIRECT),//0.5,1   
//   PID(&dic[4][currDiff], &Output[4], &targetDiff, .5, 1, 0, DIRECT),//0.5,1 
//   PID(&dic[5][currDiff], &Output[5], &targetDiff, .5, 1, 0, DIRECT),//0.5,1 
//   PID(&dic[6][currDiff], &Output[6], &targetDiff, .5, 1, 0, DIRECT)//0.5,1 
};

int WindowSize = 10;
int interval = 1000;
unsigned long windowStartTime[NUM_CELLS];

Adafruit_MPR121 cap = Adafruit_MPR121();

const uint8_t SENSE_DELAY_MS = 10;

int index = 0;
int allReached[NUM_CELLS];
unsigned long before;
unsigned long future;

void TCA9548A(uint8_t bus) {
  Wire.beginTransmission(TCAADDR);  // TCA9548A address
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();  
}

unsigned long lastButtonPress = 0;
unsigned long debounceDelay = 50;

void changeLogState(){
  unsigned long buttonPressTime = millis();
  if (buttonPressTime - lastButtonPress > debounceDelay) {
    lastButtonPress = buttonPressTime;
    logState = !logState;
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  for (int s = 0; s < NUM_CELLS; s++){
    dic[s][a_left] = as[s][0];
    dic[s][a_right] = as[s][1];
    dic[s][b_left] = bs[s][0];
    dic[s][b_right] = bs[s][1];
    dic[s][c_left] = cs[s][0];
    dic[s][c_right] = cs[s][1];
    dic[s][currDiff] = 0;
    dic[s][supplyValveA] = supplyValveAs[s];
    dic[s][exhaustValveA] = exhaustValveAs[s];
    dic[s][supplyValveB] = supplyValveBs[s];
    dic[s][exhaustValveB] = exhaustValveBs[s];
  }

  for (int i=0; i<NUM_CELLS; i++) {
    windowStartTime[i] = millis();
    PIDs[i].SetOutputLimits(-WindowSize, WindowSize);
    PIDs[i].SetMode(AUTOMATIC);
  }
  
  targetDiff = 0;
  TCA9548A(0);
  
  if (!cap.begin(0x5A)) { //checks to see if capacitive sensor board is connected, default I2C addr is 0x5A
    Serial.println("MPR121 not found, check wiring?");
    while (1);
  }
  cap.setThresholds(2, 1);
  for (int i = 0; i < NUM_CELLS; i++){
    pinMode(dic[i][supplyValveA], OUTPUT);
    pinMode(dic[i][exhaustValveA], OUTPUT);
    pinMode(dic[i][supplyValveB], OUTPUT);
    pinMode(dic[i][exhaustValveB], OUTPUT);      
  }
  for (int i = 0; i < NUM_CELLS; i++){
    digitalWrite(dic[i][supplyValveA], LOW);
    digitalWrite(dic[i][exhaustValveA], LOW);
    digitalWrite(dic[i][supplyValveB], LOW);
    digitalWrite(dic[i][exhaustValveB], LOW);      
  }
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), changeLogState, RISING);
}

void loop() {
  for (uint8_t s=0; s<NUM_CELLS; s++) {
    Serial.print("DIFF["); Serial.print(s);
    Serial.print("],");
    Serial.println();
  float input = 1.1;
  for (;;) {
    TCA9548A(0);
    if (logState){
      logAngles();
    } else {
      for (uint8_t s=0; s<NUM_CELLS; s++) {
        allReached[s] = inflationAlgo(s); 
      }
      if (state == NOT_REACHED) {
        int reached = 0;
        for (int i=0; i<NUM_CELLS; i++) {
          reached += allReached[i];
        }
        if (reached == NUM_CELLS){
          state = ALL_REACHED;
          before = millis();
          future = before + interval;
        }  
      } else if (state == ALL_REACHED) {
        if (millis() >= future) {
          state = NOT_REACHED;
          if (index < ARRAY_SIZE - 1){
            index += 1;
          } else {
            index = 0;
          }
        }
        
      } else {
        Serial.println("????");
      }
      delay(SENSE_DELAY_MS);
      }
    }
  }
}
