// hardware connection
const int CellNum = 6; // number of active cells
int readPin[CellNum] = {A0,A1,A2,A3,A4,A6}; // pin number to read the sensor signal
// pin number to control the valve, each airbag requires 4 valves to control
int bagPin[CellNum*2][2] = {{28,31}, {32,33}, {34,35}, {36,37}, {9,8}, {7,6}, {5,4}, {3,2}, {40,41}, {42,43}, {44,45}, {46,47}};

// const int CellNum = 1;
// int readPin[CellNum] = {A0};
// int bagPin[CellNum*2][2] = {{2,3},{4,5}};

// parameters for the control loop
bool useProportionalControl = true;
float prop_factor = 0.1; // the parameter for the proportional control, the larger the parameter, the quicker to reach the target (also with more overshooting)
const float tolerance = 3.0; // the tolerance to determine if the active cell reaches the target angle
uint8_t tryLimitNum; // count the number of tries
uint8_t tryLimitValue = 3; // give a limit number of tries in case never satisfied
bool tryLimitReached; // record the status of whether the tryLimitNum is reached
int minActuation = 3000; // the minimum duration for each inflation of the airbag
unsigned long actuationInverval = 100000; // the interval between each actuation, no need to change
unsigned int loopCount = 0; // record the number of looped times

bool initializeInflation = false; // whether to use the initialization function to pre-inflated all the airbags to 90degs
unsigned long constCellsPulseLength[CellNum][2]; // the array to store the constant minActuation (two airbags in one active cell are stored separately)
unsigned long cellsPulseLength[CellNum][2]; // the array to store the dynamic adjustment of the inflation duration (two airbags in one active cell are stored separately)
unsigned long lastUpdated[CellNum]; // the array to record the time when the last actuation was completed
long timeDiffs[CellNum]; // the array to record the time interval between each actuation, which is useful to observe how fast the program runs
bool actuated[CellNum]; // the array to record the status of whether the active cell has been actuated, if actuated, jump to check the next active cell
float currentAngle[CellNum]; // the array to record the current angle of the active cell
float targetAngle[CellNum]; // the array to store the target angle of the active cell
float angleDiffs[CellNum]; // the array to record the angle difference between the current angle and the target angle
float prevAngleDiffs[CellNum]; // the array to record the previous angle difference between the current angle and the target angle
bool cellsState[CellNum]; // the array to record the status of whether the active cell has reached the target angle
uint8_t numUnchangedAngleDiff[CellNum]; // the array to record the number of times that angle difference is not changed after each actuation
bool overShoot[CellNum]; // the array to record the status of whether the active cell has over-shooted the target angle
bool allReached; // record the status of whether all active cells have reached the target angle
bool updateFinished; // whether all the target angles are reached
uint8_t update; // used in loop()

// parameters for moving average filter for sensor readings
float intitialSensorError[CellNum] = {-6, 1.56, 2.37, -1, -12, 0.88}; // the initial errors caused by sensor and position of the internal scissor mechanisms
bool useRawSignal = false; // whether to use the raw signal from the sensor, if not, use the moving average filter
const int windowSize = 6; // window size for moving average filter
float sensorAngleReadings[CellNum][windowSize]; // the array to store the sensor readings for moving average filter
float readingsSum[CellNum]; // the array to store the sum of sensor readings for moving average filter
int windowIndex[CellNum]; // the array to store the index of window

// parameters for user input interaction
bool targetRecorded;
bool startReplay;
unsigned long lastRecordingTime;
uint8_t pathPointIndex = 0;
const int mPointStepSize = 1;
float mAngleSteps[mPointStepSize][CellNum];
const int mPathStepSize = 99; // number of sampling points on a user-defined path from input interaction
float collectedAngles[mPathStepSize][CellNum] = {}; // the array to store the collected angles of all active cells from input interaction

int stepIndex = 0; // the index of the starting point on the path

// angles from the software output
float waterDropGeneratedAngles[25][6] = {
  {90.0002,90.0002,90.0002,90.0002,90.0002,90.0002},
  {87.6522,87.4781,91.0195,86.6329,88.9803,86.4588},
  {84.9112,84.9112,92.1849,82.7265,87.8149,82.7265},
  {82.4423,82.4423,93.395,79.0475,86.6049,79.0475},
  {79.5569,79.5569,94.8257,74.7315,85.1748,74.7315},
  {77.0393,77.0393,96.3228,70.7162,83.677,70.7162},
  {75.274,75.274,97.8686,67.4056,82.1312,67.4056},
  {74.4198,74.4198,99.6471,64.7723,80.3527,64.7723},
  {74.4708,74.4708,101.006,63.4642,78.9937,63.4642},
  {74.9286,74.9286,102.43,62.4988,77.5705,62.4988},
  {75.6929,75.6929,103.335,62.3584,76.6652,62.3584},
  {77.0176,77.0176,104.317,62.7005,75.6826,62.7005},
  {79.6142,79.6142,105.321,64.2939,74.6793,64.2939},
  {81.7204,81.7204,105.759,65.9612,74.241,65.9612},
  {84.0414,84.0414,105.607,68.4341,74.3928,68.4341},
  {86.9355,86.9355,104.859,72.0769,75.1411,72.0769},
  {89.4072,89.4072,103.344,76.0636,76.6566,76.0636},
  {90.7187,90.7187,100.877,79.8417,79.1232,79.8417},
  {90.5823,90.5823,98.5104,82.0722,81.4895,82.0722},
  {89.3912,89.3912,97.1479,82.2429,82.852,82.2429},
  {87.4654,87.4654,95.5201,81.9456,84.4798,81.9456},
  {87.4935,87.4935,94.0304,83.4633,85.9695,83.4633},
  {88.1559,88.1559,92.429,85.7271,87.5714,85.7271},
  {89.1373,89.1373,91.0493,88.0877,88.9506,88.0877},
  {89.9418,89.9418,90.1177,89.8243,89.8828,89.8243}
};

float flowerGeneratedAngles[25][6] = {
  {89.0002,99.0002,99.0002,100.0002,92.2002,88.0002},
  {96.2155,97.8801,99.4872,106.7279,91.7126,86.3931},
  {91.9847,101.9847,102.6408,99.3436,88.5591,87.3436},
  {88.4828,98.4828,106.569,91.9141,84.6309,79.9141},
  {82.7653,92.7653,109.198,83.5678,82.0021,71.5678},
  {76.6793,86.6793,109.311,77.3678,81.8887,65.3678},
  {73.1574,83.1574,107.5275,75.63,83.6729,63.63},
  {73.7796,83.7796,105.5222,78.257,85.6777,66.257},
  {78.0017,88.0017,104.7693,83.2326,86.4311,71.2326},
  {83.623,93.623,106.035,87.5882,85.1649,75.5882},
  {87.8297,97.8297,109.037,88.792,82.1626,76.792},
  {89.2695,99.2695,112.851,86.4188,78.349,74.4188},
  {87.6681,97.6681,115.774,81.8936,75.4257,69.8936},
  {84.3644,94.3644,116.275,78.0892,74.9244,66.0892},
  {81.9626,91.9626,114.29,77.6726,76.9103,65.6726},
  {82.6324,92.6324,110.788,81.8443,80.4116,69.8443},
  {87.2739,97.2739,107.4221,89.852,83.7783,77.852},
  {93.6589,103.6589,105.4529,98.2063,85.7476,86.2063},
  {98.9307,108.9307,105.3938,103.5365,85.806,91.5365},
  {100.364,110.364,107.0887,103.2752,84.1112,91.2752},
  {97.8226,107.8226,109.351,98.4716,81.8486,86.4716},
  {92.4511,102.4511,109.964,92.4876,81.2361,80.4876},
  {87.0779,97.0779,108.1601,88.9175,83.0398,76.9175},
  {84.3031,94.3031,104.5762,89.7271,86.6236,77.7271},
  {85.5533,95.5533,100.9775,94.576,90.2229,82.576}
};

float heartGeneratedAngles[36][6] = {
  {90.0002,90.0002,90.0002,90.0002,90.0002,90.0002},
  {91.1559,91.1559,89.1746,91.9809,90.8253,91.9809},
  {94.0046,94.0046,87.1079,96.8969,92.8919,96.8969},
  {98.5275,98.5275,84.929,103.599,95.0709,103.599},
  {101.16,101.16,83.94,107.22,96.0604,107.22},
  {106.572,106.572,83.0353,113.537,96.9645,113.537},
  {109.241,109.241,83.9991,115.241,96.0008,115.241},
  {108.994,108.994,86.7458,112.248,93.254,112.248},
  {106.712,106.712,90.8316,105.881,89.1688,105.881},
  {103.203,103.203,95.4376,97.7661,84.5628,97.7661},
  {100.099,100.099,98.0549,92.044,81.945,92.044},
  {96.7766,96.7766,100.405,86.3711,79.5947,86.3711},
  {93.4042,93.4042,102.622,80.7813,77.3774,80.7813},
  {90.47,90.47,104.394,76.0756,75.6058,76.0756},
  {87.8946,87.8946,105.765,72.1297,74.2347,72.1297},
  {86.127,86.127,106.637,69.49,73.3632,69.49},
  {85.3782,85.3782,106.912,68.4667,73.0888,68.4667},
  {83.0038,83.0038,105.52,67.4835,74.4794,67.4835},
  {78.2403,78.2403,102.078,66.1629,77.9228,66.1629},
  {75.7364,75.7364,99.6884,66.0477,80.3115,66.0477},
  {73.6732,73.6732,97.1387,66.5347,82.8612,66.5347}, 
  {74.3017,74.3017,87.4133,76.8886,95.5871,76.8886},
  {75.5777,75.5777,86.1402,79.4377,100.8597,79.4377},
  {77.1631,77.1631,85.2217,81.9416,97.7787,81.9416},
  {79.1369,79.1369,84.4884,84.6488,95.5115,84.6488},
  {81.0592,81.0592,84.2088,86.8507,95.7911,86.8507},
  {83.5974,83.5974,84.105,89.4926,95.8948,89.4926},
  {85.5076,85.5076,84.5691,90.9387,95.4313,90.9387},
  {87.0002,87.0002,85.3111,91.6893,94.6887,91.6893},
  {88.1822,88.1822,86.1837,91.9981,93.8161,91.9981},
  {89.0565,89.0565,87.1979,91.8589,92.802,91.8589},
  {89.4771,89.4771,88.1942,91.2831,91.8056,91.2831},
  {89.6885,89.6885,89.1866,90.5021,90.8132,90.5021},
  {89.8375,89.8375,89.8369,90.0008,90.1635,90.0008},
  {90.0002,90.0002,90.0002,90.0002,90.0002,90.0002},
  {90.0002,90.0002,90.0002,90.0002,90.0002,90.0002}
};

float getSensorAngleInDegs(int sensorIndex) {
  // CJMCU-103 Rotary Angle Module SV01A103AEA01R00 has effective rotation angle 333.3 which can guarantee linearity
  // The relative value of 512 from the sensor indicates 90 degs
  
  // consider the intial error caused by sensor and position of the internal scissor mechanisms, and then calibrate it by the array intitialSensorError[sensorIndex]
  float incomingByte = analogRead(readPin[sensorIndex]);
  return 256.65 - (incomingByte / 1024) * 333.3 - intitialSensorError[sensorIndex];
}

float movingAverage(int sensorIndex, float newSensorAngle) {
  // A moving average filter is used to smooth the sensor readings
  readingsSum[sensorIndex] = readingsSum[sensorIndex] - sensorAngleReadings[sensorIndex][windowIndex[sensorIndex]];
  sensorAngleReadings[sensorIndex][windowIndex[sensorIndex]] = newSensorAngle;
  readingsSum[sensorIndex] = readingsSum[sensorIndex] + newSensorAngle;
  windowIndex[sensorIndex] = (windowIndex[sensorIndex]+1) % windowSize; // increase the index, and wrap to 0 if it exceeds the window size
  return readingsSum[sensorIndex] / float(windowSize);
}

void testSensorSignal() {
  // for (int i=0; i<CellNum; i++) {
  //   Serial.print("Sensor"); Serial.print(i); Serial.print(": ");
  //   float sensorAngle = getSensorAngleInDegs(i);
  //   if (useRawSignal) {
  //     Serial.print(sensorAngle);
  //   } else {
  //     Serial.println(movingAverage(i, sensorAngle));
  //   }
  //   delay(50);
  // }
  
  // Print the values of all sensor readings, can also be visualized from the serial plotter
  Serial.println();
  if (useRawSignal) {
    Serial.print("SS1:");
    Serial.println(getSensorAngleInDegs(0));
    Serial.print("SS2:");
    Serial.println(getSensorAngleInDegs(1));
    Serial.print("SS3:");
    Serial.println(getSensorAngleInDegs(2));
    Serial.print("SS4:");
    Serial.println(getSensorAngleInDegs(3));
    Serial.print("SS5:");
    Serial.println(getSensorAngleInDegs(4));
    Serial.print("SS6:");
    Serial.println(getSensorAngleInDegs(5));
  } else {
    Serial.print("ASS1:");
    Serial.println(movingAverage(0, getSensorAngleInDegs(0)));
    Serial.print("ASS2:");
    Serial.println(movingAverage(1, getSensorAngleInDegs(1)));
    Serial.print("ASS3:");
    Serial.println(movingAverage(2, getSensorAngleInDegs(2)));
    Serial.print("ASS4:");
    Serial.println(movingAverage(3, getSensorAngleInDegs(3)));
    Serial.print("ASS5:");
    Serial.println(movingAverage(4, getSensorAngleInDegs(4)));
    Serial.print("ASS6:");
    Serial.println(movingAverage(5, getSensorAngleInDegs(5)));
  }
  delay(200);
}

void testSensorSignal(int sensorIndex) {
  // input the index of the sensor and print its raw sensor reading
  Serial.print("Sensor "); Serial.print(sensorIndex); Serial.print(": ");
  float incomingByte = analogRead(readPin[sensorIndex]);
  float calibratedAngle = 256.65 - (incomingByte / 1024) * 333.3;
  Serial.println(calibratedAngle);
  delay(200);
  Serial.println();
}

void inflate(int index) {
  digitalWrite(bagPin[index][0], LOW);
  digitalWrite(bagPin[index][1], HIGH);
}

void lock(int index) {
  digitalWrite(bagPin[index][0], LOW);
  digitalWrite(bagPin[index][1], LOW);
}

void vent(int index) {
  digitalWrite(bagPin[index][0], HIGH);
  digitalWrite(bagPin[index][1], LOW);
}

void ventAllAirbags() {
  // vent all airbags
  for (int i=0; i<CellNum; i++) {
    uint8_t cellLeftBagIndex = i * 2;
    uint8_t cellRightBagIndex = i * 2 + 1;
    vent(cellLeftBagIndex);
    vent(cellRightBagIndex);
  }
}

void initializeAirbags(int start, int end) {
  // pre-inflated all the airbags and adjusted all the active cells to 90degs
  // need the start index and the end index of cells
  for (int i=start; i<end; i++) {
    uint8_t cellLeftBagIndex = i * 2;
    uint8_t cellRightBagIndex = i * 2 + 1;
    
    // inflate all the airbags
    int num = 2;
    for (int j=0; j<num; j++) {
      inflate(cellLeftBagIndex);
      delayMicroseconds(6000);
      lock(cellLeftBagIndex);
      inflate(cellRightBagIndex);
      delayMicroseconds(6000);
      lock(cellRightBagIndex);
      delay(actuationInverval/1000);
    }
  }
  delay(500);
  
  // adjust all the active cells to 90degs to finish the initialization
  // specify the target angle of 90degs
  while(!allReached) { specificAngleUpdate(start, end, 90.0); }
  
  initializeInflation = true;
  for (int i=start; i<end; i++) { prevAngleDiffs[i] = 0; }
  Serial.println("Initialization is finished"); Serial.println();
}

void specificAngleUpdate(int start, int end, float target) {
  // update the active cells to a specfic target angle (a more simple version without proportional control and the detection of overshooting)
  for (int i=start; i<end; i++) {
    unsigned long currentTime = micros();
    
    currentAngle[i] = movingAverage(i, getSensorAngleInDegs(i)); // get smooth sensor signal using moving average filter
    targetAngle[i] = target; // get the target angle
    angleDiffs[i] = targetAngle[i] - currentAngle[i]; // get the angle difference
    
    unsigned long actuationTime; // initialize this variable every loop to dynamically adjust the inflation duration
    
    // adjust the inflation duration after the angle difference is not changed anymore
    if (angleDiffs[i] > 0) {
      // update the actuationTime of the right airbag
      if (numUnchangedAngleDiff[i] >= 3) {
        
        // increasing the inflation duration by a constant value
        cellsPulseLength[i][1] += 200;
        numUnchangedAngleDiff[i] = 0;
      }
      actuationTime = cellsPulseLength[i][1];
      
    } else {
      // update the actuationTime of the left airbag
      if (numUnchangedAngleDiff[i] >= 3) {
        
        // increasing the inflation duration by a constant value
        cellsPulseLength[i][0] += 200;
        numUnchangedAngleDiff[i] = 0;
      }
      actuationTime = cellsPulseLength[i][0];
    }
    
    // if it is not the time to update the active cell, continue the loop to the next active cell
    if (currentTime < actuationTime + lastUpdated[i]) { continue; }
    
    // if it is the time to update the active cell, update the lastUpdated time with the current time
    lastUpdated[i] = currentTime;
    
    // get correct index of the valves
    uint8_t cellLeftBagIndex = i * 2;
    uint8_t cellRightBagIndex = i * 2 + 1;
    
    // if the active cell has not reached the target angle while also not during actuation, update the airbag
    if (!actuated[i] && abs(angleDiffs[i]) > tolerance) {
      actuated[i] = true;
      
      // inflate the airbag to adjust the angle of active cells
      // if inside the initialization process, the airbag is not vented
      if (angleDiffs[i] < 0) {
        if (initializeInflation) { vent(cellRightBagIndex); }
        inflate(cellLeftBagIndex);
      } else {
        if (initializeInflation) { vent(cellLeftBagIndex); }
        inflate(cellRightBagIndex);
      }
    
    // if the active cell has reached the target angle or during actuation
    } else {
      // lock its airbag anyway for minimum actuation everytime
      lock(cellLeftBagIndex);
      lock(cellRightBagIndex);
      
      // wait for a period for next control (important, thus needs actuated[i] to check)
      lastUpdated[i] += actuationInverval;
      actuated[i] = false;
      
      // check the angle difference to see what happened after the last actuation
      if (abs(angleDiffs[i]) > tolerance) {

        // record the number that the angle difference is not changed after the last actuation
        if (abs(abs(angleDiffs[i]) - abs(prevAngleDiffs[i])) < 0.5) { // =0 or <1 or <0.5, 0.5 is an appropriate parameter
          numUnchangedAngleDiff[i] += 1;
        } else {
          numUnchangedAngleDiff[i] = 0;
        }
      }

      prevAngleDiffs[i] = angleDiffs[i]; // update the previous angle difference with the current angle difference
    }
  }
  
  // after each forloop, record the status of all active cells with current sensor reading
  for (int j=start; j<end; j++) {
    currentAngle[j] = movingAverage(j, getSensorAngleInDegs(j));
    angleDiffs[j] = targetAngle[j] - currentAngle[j];
    cellsState[j] = (abs(angleDiffs[j]) < tolerance) ? 1 : 0;
  }
  
  // print the angle difference of all active cells every 500 forloops
  loopCount++;
  if ((loopCount % 500) == 0) {
    for (int j=start; j<end; j++) {
      Serial.print("AD"); Serial.print(j); Serial.print(": ");
      Serial.print(currentAngle[j]); Serial.print("; ");
    }
    Serial.println();
    loopCount = 0;
  }
  
  allReached = true;
  for (int j=start; j<end; j++) {
    if (cellsState[j] == false) {
      allReached = false;
      break;
    }
  }
  
  if (allReached) {
    // lock all airbags at first in case over-shooting
    for (int j=start; j<end; j++) {
      uint8_t cellLeftBagIndex = j * 2;
      uint8_t cellRightBagIndex = j * 2 + 1;
      lock(cellLeftBagIndex);
      lock(cellRightBagIndex);
      Serial.print("Reached to the target with: ");
      Serial.println(currentAngle[j]);
    }
    
    // if not in the initializeInflation process, maintain the target angle for a while after reaching
    if (initializeInflation) { delay(30000); }
    
    // reset all the actuation duration and then maintain the vent state for a while and then exit
    for (int j=start; j<end; j++) {
      cellsPulseLength[j][0] = constCellsPulseLength[j][0];
      cellsPulseLength[j][1] = constCellsPulseLength[j][1];
      uint8_t cellLeftBagIndex = j * 2;
      uint8_t cellRightBagIndex = j * 2 + 1;
      if (initializeInflation) {
        vent(cellLeftBagIndex);
        vent(cellRightBagIndex);
      }
    }
    if (initializeInflation) { 
      delay(300000); 
      exit(0);
    } 
  }
}

void unifiedUpdate(float angleSteps[][CellNum], int stepSize, int waiting) {
  // update all the active cells simultaneously
  for (int i=0; i<CellNum; i++) {
    unsigned long currentTime = micros();
    // timeDiffs[i] = (long)currentTime - lastUpdated[i]; // calculate the duration of single forloop (unit is microsecond)
    
    currentAngle[i] = movingAverage(i, getSensorAngleInDegs(i)); // get smooth sensor signal using moving average filter
    targetAngle[i] = angleSteps[stepIndex][i]; // get the target angle
    angleDiffs[i] = targetAngle[i] - currentAngle[i]; // get the angle difference
    
    unsigned long actuationTime; // initialize this variable every loop to dynamically adjust the inflation duration
    
    // adjust the inflation duration after the angle difference is not changed anymore
    if (angleDiffs[i] > 0) {
      // update the actuationTime of the right airbag
      if (numUnchangedAngleDiff[i] >= 3) {
        
        if (useProportionalControl) {
          if (abs(angleDiffs[i]) > 3.0) {
            // use proportional control
            float prop_p = (abs(angleDiffs[i]) * prop_factor + abs(currentAngle[i])) / abs(currentAngle[i]); 
            cellsPulseLength[i][1] = cellsPulseLength[i][1] * prop_p;
            
          } else {
            // not like proportional control but increasing the inflation duration by a constant value
            cellsPulseLength[i][1] += 100;
          }
          numUnchangedAngleDiff[i] = 0;
          
        } else {
          // don't use proportional control
          cellsPulseLength[i][1] += 200;
          numUnchangedAngleDiff[i] = 0;
        }
      }
      
      actuationTime = cellsPulseLength[i][1]; // update the actuationTime of the right airbag
      
    } else {
      // update the actuationTime of the left airbag
      if (numUnchangedAngleDiff[i] >= 3) {  
        
        if (useProportionalControl) {
          if (abs(angleDiffs[i]) > 3.0) {
            // use proportional control
            float prop_p = (abs(angleDiffs[i]) * prop_factor + abs(currentAngle[i])) / abs(currentAngle[i]); // proportional control
            cellsPulseLength[i][0] = cellsPulseLength[i][0] * prop_p;
          
          } else {
            // not like proportional control but increasing the inflation duration by a constant value
            cellsPulseLength[i][0] += 100;
          }
          numUnchangedAngleDiff[i] = 0;
          
        } else {
          // don't use proportional control
          cellsPulseLength[i][0] += 200;
          numUnchangedAngleDiff[i] = 0;
        }
      }

      actuationTime = cellsPulseLength[i][0]; // update the actuationTime of the left airbag
    }
    
    // if it is not the time to update the active cell, continue the loop to the next active cell
    if (currentTime < actuationTime + lastUpdated[i]) { continue; }
    
    // if it is the time to update the active cell, update the lastUpdated time with the current time
    lastUpdated[i] = currentTime;
    
    // get correct index of the valves
    uint8_t cellLeftBagIndex = i * 2;
    uint8_t cellRightBagIndex = i * 2 + 1;
    
    // if the active cell has not reached the target angle while also not during actuation, update the airbag
    if (!actuated[i] && abs(angleDiffs[i]) > tolerance) {
      actuated[i] = true;
      
      // inflate the airbag to adjust the angle of active cells
      // however, if overshooting is detected, only vent the airbag to ensure the angle is changed gradually
      if (angleDiffs[i] < 0) {
        vent(cellRightBagIndex);
        if (!overShoot[i]) {
          inflate(cellLeftBagIndex);
        }
      } else {
        vent(cellLeftBagIndex);
        if (!overShoot[i]) {
          inflate(cellRightBagIndex);
        }
      }
      
    // if the active cell has reached the target angle or during actuation, lock its airbag anyway for minimum actuation everytime
    } else {
      if (overShoot[i]) {
        // if overshooting, lock two airbags after each actuation for slow movement
        lock(cellLeftBagIndex);
        lock(cellRightBagIndex);
      } else {
        // if not overshooting, only lock the actuated airbag, in case vent never happens and the angle is hard to be changed
        if (angleDiffs[i] < 0) {
          lock(cellLeftBagIndex);
        } else {
          lock(cellRightBagIndex);
        }
      }
      
      // wait for a period for next control to make the whole system more stable (important, thus needs actuated[i] to check)
      lastUpdated[i] += actuationInverval;
      actuated[i] = false;
      
      // check the angle difference to see what happened after the last actuation
      if (abs(angleDiffs[i]) > tolerance) {
        
        // record the number that the angle difference is not changed after the last actuation
        if (abs(abs(angleDiffs[i]) - abs(prevAngleDiffs[i])) < 0.5) { // =0 or <1 or <0.5, 0.5 is an appropriate parameter
          numUnchangedAngleDiff[i] += 1;
        } else {
          numUnchangedAngleDiff[i] = 0;
        }
        
        // check if overshooting happens
        if (prevAngleDiffs[i] < 0 && angleDiffs[i] > 0 || prevAngleDiffs[i] > 0 && angleDiffs[i] < 0) {
          overShoot[i] = true; // set the status of overshooting to true
          
          // since overshooting happens, reset the actuation duration to its minimum for slow movement
          // if the angle is changed dramatically, the active cell will keeps overshooting and never reaches the target angle
          cellsPulseLength[i][0] = constCellsPulseLength[i][0];
          cellsPulseLength[i][1] = constCellsPulseLength[i][1];
          numUnchangedAngleDiff[i] = 0;
        }
        
      } else {
        // even if only one active has reached, reset its actuation duration in case overshooting happens in next actuation
        cellsPulseLength[i][0] = constCellsPulseLength[i][0];
        cellsPulseLength[i][1] = constCellsPulseLength[i][1];
        numUnchangedAngleDiff[i] = 0;
        
        overShoot[i] = false; // reset overshooting to false
      }
      
      prevAngleDiffs[i] = angleDiffs[i]; // update the previous angle difference with the current angle difference
    }
  }
  
  // after each forloop, record the status of all active cells with current sensor reading
  for (int j=0; j<CellNum; j++) {
    currentAngle[j] = movingAverage(j, getSensorAngleInDegs(j));
    angleDiffs[j] = targetAngle[j] - currentAngle[j];
    cellsState[j] = (abs(angleDiffs[j]) < tolerance) ? 1 : 0;
  }
  
  // print the angle difference of all active cells every 100 forloops
  loopCount++;
  if ((loopCount % 100) == 0) {
    // for (int j=0; j<CellNum; j++) {
    //   Serial.print("AD"); Serial.print(j); Serial.print(": ");
    //   Serial.print(currentAngle[j]); Serial.print("; ");
      
    //   // Serial.print(j); Serial.print(" timeDiff "); Serial.println(timeDiffs[j]);
    // }
    // Serial.println();
    
    Serial.print("AD1: ");
    Serial.print(angleDiffs[0]);
    Serial.print(", AD2: ");
    Serial.print(angleDiffs[1]);
    Serial.print(", AD3: ");
    Serial.print(angleDiffs[2]);
    Serial.print(", AD4: ");
    Serial.print(angleDiffs[3]);
    Serial.print(", AD5: ");
    Serial.print(angleDiffs[4]);
    Serial.print(", AD6: ");
    Serial.println(angleDiffs[5]);
    
    loopCount = 0;
    tryLimitNum += 1;
    if (tryLimitNum == tryLimitValue) { tryLimitReached = true; } // check if the try limit is reached
  }
  
  allReached = true;
  for (int j=0; j<CellNum; j++) {
    if (cellsState[j] == false) {
      allReached = false;
      break;
    }
  }
  
  if (allReached) {
    // lock all airbags at first in case over-shooting
    for (int j=0; j<CellNum; j++) {
      uint8_t cellLeftBagIndex = j * 2;
      uint8_t cellRightBagIndex = j * 2 + 1;
      lock(cellLeftBagIndex);
      lock(cellRightBagIndex);
    }
    
    // reset all the actuation duration to its minimum for slow movement
    for (int j=0; j<CellNum; j++) {
      cellsPulseLength[j][0] = constCellsPulseLength[j][0];
      cellsPulseLength[j][1] = constCellsPulseLength[j][1];
      overShoot[j] = false; // reset overShoot status
    }
    
    Serial.print("Point "); Serial.print(stepIndex); Serial.println(" is satisfied!"); Serial.println();
    
    // print the angle difference after reaching the target point
    for (int j=0; j<CellNum; j++) { Serial.print(movingAverage(j, getSensorAngleInDegs(j))); Serial.print("; "); }
    Serial.println();
    stepIndex += 1;
    tryLimitNum = 0;
    delay(waiting);
    
  } else if (tryLimitReached) {
    // lock all airbags at first in case over-shooting
    for (int j=0; j<CellNum; j++) {
      uint8_t cellLeftBagIndex = j * 2;
      uint8_t cellRightBagIndex = j * 2 + 1;
      lock(cellLeftBagIndex);
      lock(cellRightBagIndex);
    }
    
    // reset all the actuation duration to its minimum for slow movement
    for (int j=0; j<CellNum; j++) {
      cellsPulseLength[j][0] = constCellsPulseLength[j][0];
      cellsPulseLength[j][1] = constCellsPulseLength[j][1];
      overShoot[j] = false; // reset overShoot status
    }
    
    Serial.print("The point "); Serial.print(stepIndex); Serial.println(" is passed");
    stepIndex += 1;
    tryLimitReached = false;
    tryLimitNum = 0;
    delay(waiting);
  }
  
  // all the points are reached, stop the movement
  if (stepIndex == stepSize) {
    Serial.println("All points have been reached!");
    updateFinished = true;
    // exit(0);
  }
}

void manualInputPoint(bool loopMovement) {
  if (startReplay) {
    // start reach to the target point
    if (!updateFinished) {
      unifiedUpdate(mAngleSteps, mPointStepSize, 30000);
    }
    // whether to loop the movement
    if (loopMovement) {
      if (updateFinished) {
        ventAllAirbags();
        delay(500);
        updateFinished = false;
        stepIndex = 0;
      }
    }
    
  // get user input of one target point
  } else {
    if (!targetRecorded) {
      while (Serial.available() == 0) {
        Serial.println("Please move the grid corner to the target point, and input 1 to store the data");
        delay(1000);
        testSensorSignal();
      }
    }
    // get the user input
    if (Serial.available()) {
      char input = Serial.read();
      switch (input) {
        case '0':
          targetRecorded = false;
          break;
        case '1':
          Serial.println("The data is stored, enter 0 to restore the data");
          for (int i=0; i<CellNum; i++) {
            mAngleSteps[0][i] = getSensorAngleInDegs(i);
            Serial.print(mAngleSteps[0][i]); Serial.print("; ");
          }
          Serial.println();
          targetRecorded = true;
          
          Serial.println("Move the grid to its default position, and input 2 to replay the movement");
          break;
        case '2':
          if (targetRecorded) {
            if (mAngleSteps[0][0] != 0) {
              Serial.println("\n");
              Serial.println("Start to replay the movement");
              startReplay = true;
            }
          } else {
            Serial.println("Please store the data first");
            delay(1000);
          }
          break;
      }
    } 
  }
}

void manualInputPath(unsigned long pathRecordingInterval) {
  if (targetRecorded) {
    // sample a point every time interval while the user moving the grid
    unsigned long currentTime = millis();
    if (currentTime - lastRecordingTime > pathRecordingInterval) {
      lastRecordingTime = currentTime;
      pathPointIndex++;
      // Serial.print("P"); Serial.print(pathPointIndex); Serial.print(": ");
      Serial.print("{");
      for (int i=0; i<CellNum; i++) {
        Serial.print(getSensorAngleInDegs(i)); 
        if (i != (CellNum - 1)) { Serial.print(", "); }
      }
      Serial.print("},");
      Serial.println();
    }
  
  // get user input of one user-defined path
  } else {
    while (Serial.available() == 0) {
      if (pathPointIndex == 0) {
        Serial.println("Input 1 to start recording angles");
        delay(1000);
      }
    }
  }
  // get the user input
  if (Serial.available()) {
    char input = Serial.read();
    switch (input) {
      case '0':
        targetRecorded = false;
        break;
      case '1':
        Serial.println(); Serial.println("Start recording data... Enter 2 to stop");
        targetRecorded = true;
        break;
      case '2':
        Serial.println(); Serial.println("Recording is stoped!");
        targetRecorded = false;
        break;
    }
  }
}

void setup() {
  // The program needs to run as quickly as possible for accurarte timing
  // Therefore, use a very high baud rate for quicker serial communication which usually takes long time to finish
  Serial.begin(2000000);
  
  // set sensor signal pin as INPUT
  for (int i=0; i<CellNum; i++) {
    pinMode(readPin[i], INPUT);
  }
  
  // set valve control pin as OUTPUT
  for (int i=0; i<CellNum*2; i++) {
    pinMode(bagPin[i][0], OUTPUT);
    digitalWrite(bagPin[i][0], LOW);
    pinMode(bagPin[i][1], OUTPUT);
    digitalWrite(bagPin[i][1], LOW);
  }
  
  // adapted to 13V power supply, 20~30 psi air pressure
  for (int i=0; i<CellNum; i++) {
    constCellsPulseLength[i][0] = minActuation;
    constCellsPulseLength[i][1] = minActuation;
    cellsPulseLength[i][0] = minActuation;
    cellsPulseLength[i][1] = minActuation;
  }
  
  // get initial sensor readings for the moving average filter function
  if (!useRawSignal) {
    unsigned currentTime = millis();
    while (millis() - currentTime < 100) {
      for (int i=0; i<CellNum; i++) {
        currentAngle[i] = movingAverage(i, getSensorAngleInDegs(i));
      }
    }
  } else {
    delay(100);
  }
}

void loop() {
  // specify the update mode to excute different functions
  update = 4;
  
  if (update == 1) {
    // draw a path according to the software output (e.g., heart symbol)
    if (updateFinished) {
      // testSensorSignal();
    } else {
      unifiedUpdate(waterDropGeneratedAngles, 25, 0);
    }
    
  } else if (update == 2) {
    // update the selected active cells to a specific target angle
    int start = 0;
    int end = CellNum;
    
    // if (!initializeInflation) {
    //   delay(1000);
    //   initializeAirbags(start, end);
    // }
    initializeInflation = true;
    specificAngleUpdate(start, end, 75.0);
    
  } else if (update == 3) {
    // int evalCell = 0;
    // testSensorSignal(evalCell);
    testSensorSignal();
    
  } else if (update == 4) {
    ventAllAirbags();
    
  } else if (update == 5) {
    // The user inputs a point by moving the grid corner manually
    bool loopMovement = false;
    manualInputPoint(loopMovement);
    
  } else if (update == 6) {
    // The user inputs a path with pathRecordingInterval as the time interval to sample points
    manualInputPath(500);
    
    // comment out the following code first to run manualInputPath(500)
    // uncomment the following code to update the recorded path, also need to comment the manualInputPath(500)
    // if (!updateFinished) {
    //   unifiedUpdate(collectedAngles, mPathStepSize, 50);
    // }
  }
}
