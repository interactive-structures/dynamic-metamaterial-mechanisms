#include <Wire.h> //this is used to communication with the multiplexer
#include "Adafruit_MPR121.h" // this is the capacitive sensor board library
#define TCAADDR 0x70 //this is the multiplexer
#include <Adafruit_Sensor.h> //the is used for the capacitive sensor
#include <PID_v1.h> //this is the standard PID library


// number of sensors we will report on (1..12)
const uint8_t NUM_CELLS = 1;
const int ARRAY_SIZE = 4;
int sensor_pairs[6][2] = {{0,1}, {2,3}, {4,5}, {6,7}, {8,9}, {10,11}};

int CAP=0; int k_left=1; int k_right=1; int b_left=2; int b_right=2; int currDiff=3;int valveA=4; int valveB=5;
double dic[NUM_CELLS][9];

//sensor constants from spreadsheet.
//With the sensor oriented like this: |∩|, list the constants for the left sensor, then the right.
//constants will be paired (k0, k1), (k2, k3), etc.
double k0 = 1.05; double b0 = 32.3;
double k1 = 1.3; double b1 = 23.7;

//arrays of sensor constants and valve pins (matched by index).
//pair the left and right sensor of each cell (when sensor is oriented like this : |∩|)
double ks[][2] =      {{k0, k1}};
double bs[][2] =      {{b0, b1}};
double valveAs[] = {7};
double valveBs[] = {10};

//creates an enumerated datatype, "states"
enum states {
  NOT_REACHED,
  ALL_REACHED
};

//instantiate a states object
enum states state;

//Define Variables we'll be connecting to
double Output[NUM_CELLS];
double targetDiff;

//Specify the links and initial tuning parameters
//PID myPID1(&diff, &Output, &targetDiff, 1, 3, 0, DIRECT);
PID PIDs[] = {
   PID(&dic[0][currDiff], &Output[0], &targetDiff, .5, 1, 0, DIRECT),//0.5,1
//   PID(&dic[1][currDiff], &Output[1], &targetDiff, 2, 10, 0, DIRECT), //0.5,1
//   PID(&dic[2][currDiff], &Output[2], &targetDiff, .5, 1, 0, DIRECT),//0.5,1
//   PID(&dic[3][currDiff], &Output[3], &targetDiff, .5, 1, 0, DIRECT),//0.5,1   
//   PID(&dic[4][currDiff], &Output[4], &targetDiff, .5, 1, 0, DIRECT),//0.5,1 
//   PID(&dic[5][currDiff], &Output[5], &targetDiff, .5, 1, 0, DIRECT),//0.5,1 
//   PID(&dic[6][currDiff], &Output[6], &targetDiff, .5, 1, 0, DIRECT)//0.5,1 
};

int WindowSize = 50;
int interval = 50;

unsigned long windowStartTime[NUM_CELLS];
unsigned long before;
unsigned long future;

// object for cap touch sensor board
Adafruit_MPR121 cap = Adafruit_MPR121();

// extra time we will delay after each read and report
const uint8_t SENSE_DELAY_MS = 10;

// buffers for past values from each sensor
const uint16_t BUFFER_SIZE = 50; 
uint16_t buffer[NUM_CELLS][BUFFER_SIZE];
static int buffer_next = 0;
static int buffer_cur = 0;
static uint32_t buffer_sum[NUM_CELLS];


// number of sensor values to read and discard at start
const uint8_t STARTING_IGNORE = 50;

int index = 0;
int allReached[NUM_CELLS];

void TCA9548A(uint8_t bus) {
  Wire.beginTransmission(TCAADDR);  // TCA9548A address
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();  
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  //populate dic with sensor constants, valve pins, default currDiff
  for (int s = 0; s < NUM_CELLS; s++){
        dic[s][k_left] = ks[s][0];
        dic[s][k_right] = ks[s][1];
        dic[s][b_left] = bs[s][0];
        dic[s][b_right] = bs[s][1];
        dic[s][currDiff] = 0;
        dic[s][valveA] = valveAs[s];
        dic[s][valveB] = valveBs[s];
  }

  //start the PID windows (this is their 'memory')
  for (int i=0; i<NUM_CELLS; i++) {
    windowStartTime[i] = millis();
    PIDs[i].SetOutputLimits(0, WindowSize);
    PIDs[i].SetMode(AUTOMATIC);
  }

  targetDiff = 0;

  TCA9548A(0);

  //checks to see if capacitive sensor board is connected
  // Default I2C addr is 0x5A
  if (!cap.begin(0x5A)) {
    Serial.println("MPR121 not found, check wiring?");
    while (1);
  }

  //I don't know why the values are 2 and 1
  cap.setThresholds(2, 1);

  // Set all the motor control pins to outputs
  pinMode(7, OUTPUT);
  pinMode(10, OUTPUT);
               
  digitalWrite(7, LOW); //valve closed
  digitalWrite(10, LOW); //valve closed
}


void loop() {
  uint16_t val, old_val;

  TCA9548A(0);
  // read a few values to get past any early anomalies
  for (uint8_t cnt=0; cnt<STARTING_IGNORE; cnt++) {
    for (uint8_t s=0; s<NUM_CELLS; s++) {
      buffer[s][0] = val = cap.filteredData(s);
    }
  }
  
  // fill the buffers with the last of those values
  for (uint8_t s=0; s<NUM_CELLS; s++) {
    //haven't we done this already?
    val = buffer[s][0];
    buffer_sum[s] = val;
    for (uint8_t i=1; i<BUFFER_SIZE; i++) {
      buffer[s][i] = val;
      buffer_sum[s] += val;
    }
  }
  
  // text to create a legend for the plot
  for (uint8_t s=0; s<NUM_CELLS; s++) {
    Serial.print("DIFF["); Serial.print(s);
    Serial.print("],");
  }
  Serial.println();
  float input = 1.1;
  
  for (;;) {
    TCA9548A(0);
    
    // read each sensor and put the value in its buffer
    for (uint8_t s=0; s<NUM_CELLS; s++) {
      old_val = buffer[s][buffer_next];
      val = cap.filteredData(sensor_pairs[s][crossover(s, sensor_pairs)]); 
      buffer[s][buffer_next] = val;
      
      buffer_sum[s] -= old_val;
      buffer_sum[s] += val;
    }

    // advance position in buffers
    buffer_cur = buffer_next;
    buffer_next++;
    if (buffer_next >= BUFFER_SIZE) buffer_next = 0;

    for (uint8_t s=0; s<NUM_CELLS; s++) {
      dic[s][CAP] = buffer_sum[s] / BUFFER_SIZE;
      allReached[s] = inflate(s, dic[s][CAP]); 
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

    // put a delay so it isn't overwhelming
    delay(SENSE_DELAY_MS);
  }
}
