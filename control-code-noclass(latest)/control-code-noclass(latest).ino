const int CellNum = 6;
int readPin[CellNum] = {A0,A1,A2,A4,A3,A6};
int bagPin[CellNum*2][2] = {
  {28,31}, {32,33},
  {9,8}, {7,6},
  {5,4}, {3,2},
  {40,41}, {42,43},
  {44,45}, {46,47},
  {34,35}, {36,37}
};

const float tolerance = 1.0; // adjust this for application
bool initializeInflation = false;
unsigned long constCellsPulseLength[CellNum][2];
unsigned long cellsPulseLength[CellNum][2];
int minActuation = 3500; // adjust this for application
unsigned long actuationInverval = 100000;
unsigned int loopCount = 0;
unsigned long lastUpdated[CellNum];
long timeDiffs[CellNum];
bool actuated[CellNum];
bool cellsState[CellNum];
float angleDiffs[CellNum];
float prevAngleDiffs[CellNum];
float targetAngle[CellNum];
float currentAngle[CellNum];
uint8_t numUnchanged = 0;
uint8_t numUnchangedAngleDiff[CellNum];
bool overShoot[CellNum];
uint8_t update;
bool allReached;
bool updateFinished;
uint8_t tryLimitNum;
uint8_t tryLimitValue = 10;
bool tryLimitReached; // give a limit tries in case never satisfied
const int pathStepSize = 99; // number of points on the path defined in software

// moving average filter for sensor readings
bool useRawSignal = false;
const int windowSize = 6;
float sensorAngleReadings[CellNum][windowSize];
float readingsSum[CellNum];
int windowIndex[CellNum];

// user input interaction
bool targetRecorded;
bool startReplay;
unsigned long lastRecordingTime;
uint8_t pathPointIndex = 0;
const int mPointStepSize = 1;
float mAngleSteps[mPointStepSize][CellNum];
const int mPathStepSize = 25; // number of sampling points on a user-defined path

// control parameters
bool useProportionalControl = false;
float prop_factor = 0.1; // use 1.01 for best performance

int stepIndex = 0;
float collectedAngles[50][CellNum] = {};

float waterDropGeneratedAngles[mPathStepSize][6] = {
  {88.0002,100.0002,100.0002,99.0002,91.2002,89.3002},
  {85.6522,97.4781,101.0195,95.6329,90.1803,85.7588},
  {82.9112,94.9112,102.1849,91.7265,89.0149,82.0265},
  {80.4423,92.4423,103.395,88.0475,87.8049,78.3475},
  {77.5569,89.5569,104.8257,83.7315,86.3748,74.0315},
  {75.0393,87.0393,106.3228,79.7162,84.877,70.0162},
  {73.274,85.274,107.8686,76.4056,83.3312,66.7056},
  {65.4198,84.4198,109.6471,73.7723,81.5527,64.0723},
  {65.4708,84.4708,111.006,72.4642,80.1937,62.7642},
  {65.9286,84.9286,112.43,71.4988,78.7705,61.7988},
  {73.6929,85.6929,113.335,71.3584,77.8652,61.6584},
  {75.0176,87.0176,114.317,71.7005,76.8826,62.0005},
  {77.6142,89.6142,115.321,73.2939,75.8793,63.5939},
  {79.7204,91.7204,115.759,74.9612,75.441,65.2612},
  {82.0414,94.0414,115.607,77.4341,75.5928,67.7341},
  {84.9355,96.9355,114.859,81.0769,76.3411,71.3769},
  {87.4072,99.4072,113.344,85.0636,77.8566,75.3636},
  {88.7187,100.7187,110.877,88.8417,80.3232,79.1417},
  {88.5823,100.5823,108.5104,91.0722,82.6895,81.3722},
  {87.3912,99.3912,107.1479,91.2429,84.052,81.5429},
  {85.4654,97.4654,105.5201,90.9456,85.6798,81.2456},
  {85.4935,97.4935,104.0304,92.4633,87.1695,82.7633},
  {86.1559,98.1559,102.429,94.7271,88.7714,85.0271},
  {87.1373,99.1373,101.0493,97.0877,90.1506,87.3877},
  {87.9418,99.9418,100.1177,98.8243,91.0828,89.1243}
};

float flowerGeneratedAngles[mPathStepSize][6] = {
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

float heartGeneratedAngles[26][6] = {
  {87.0002,91.0002,101.0002,99.5002,89.0002,88.0002},
  {88.1559,92.1559,100.1746,101.4809,89.8253,89.9809},
  {91.0046,95.0046,98.1079,106.3969,91.8919,94.8969},
  {95.5275,99.5275,95.929,113.099,94.0709,101.599},
  {100.936,104.936,94.1786,120.257,95.8218,108.757},
  {105.252,109.252,94.1969,124.556,95.8029,113.056},
  {106.517,110.517,96.1203,123.897,93.8801,112.397},
  {105.168,109.168,99.5793,119.088,90.4206,107.588},
  {102.168,106.168,104.0478,111.621,85.9526,100.121},
  {97.099,101.099,109.0549,101.544,80.945,90.044},
  {90.4042,94.4042,113.622,90.2813,76.3774,78.7813},
  {84.8946,88.8946,116.765,81.6297,73.2347,70.1297},
  {82.3782,86.3782,117.912,77.9667,72.0888,66.4667},
  {80.0038,84.0038,116.52,76.9835,73.4794,65.4835},
  {75.2403,79.2403,113.078,75.6629,76.9228,64.1629},
  {70.6732,74.6732,108.1387,76.0347,81.8612,64.5347},
  {69.1314,73.1314,103.5436,79.0874,86.4563,67.5874},
  {70.2211,74.2211,99.86,83.8613,90.1404,72.3613},
  {72.5777,76.5777,97.1402,88.9377,92.8597,77.4377},
  {76.1369,80.1369,95.4884,94.1488,94.5115,82.6488},
  {80.5974,84.5974,95.105,98.9926,94.8948,87.4926},
  {84.0002,88.0002,96.3111,101.1893,93.6887,89.6893},
  {86.0565,90.0565,98.1979,101.3589,91.802,89.8589},
  {86.6885,90.6885,100.1866,100.0021,89.8132,88.5021},
  {87.0002,91.0002,101.0002,99.5002,89.0002,88.0002},
  {100.0002,105.0002,102.0002,103.5002,89.0002,90.0002}
};

float getSensorAngleInDegs(int sensorIndex) {
  // CJMCU-103 Rotary Angle Module SV01A103AEA01R00 has effective rotation angle 333.3 which can guarantee linearity
  // 512 indicates 90 degs
  
  float incomingByte = analogRead(readPin[sensorIndex]);
  return (incomingByte / 1024) * 333.3 - 76.65;
  
  // if (sensorIndex == 3) {
  //   float incomingByte = analogRead(readPin[sensorIndex]);
  //   return 256.65 - (incomingByte / 1024) * 333.3;
  // } else {
  //   float incomingByte = analogRead(readPin[sensorIndex]);
  //   return (incomingByte / 1024) * 333.3 - 76.65;
  // }
}

float movingAverage(int sensorIndex, float newSensorAngle) {
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
  Serial.print("Sensor "); Serial.print(sensorIndex); Serial.print(": ");
  float incomingByte = analogRead(readPin[sensorIndex]);
  float calibratedAngle = (incomingByte / 1024) * 333.3 - 76.65;
  // float calibratedAngle = 256.65 - (incomingByte / 1024) * 333.3;
  Serial.println(calibratedAngle);
  delay(500);
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
  for (int i=0; i<CellNum; i++) {
    uint8_t cellLeftBagIndex = i * 2;
    uint8_t cellRightBagIndex = i * 2 + 1;
    vent(cellLeftBagIndex);
    vent(cellRightBagIndex);
  }
}

void separateUpdate(int start, int end, float target) {
  for (int i=start; i<end; i++) {
    unsigned long currentTime = micros();
    unsigned long actuationTime;
    
    if (angleDiffs[i] > 0) {
      if (numUnchangedAngleDiff[i] >= 2) {
        cellsPulseLength[i][1] += 100;
        numUnchangedAngleDiff[i] = 0;
      }
      actuationTime = cellsPulseLength[i][1];
      
    } else {
      if (numUnchangedAngleDiff[i] >= 2) {
        cellsPulseLength[i][0] += 100;
        numUnchangedAngleDiff[i] = 0;
      }
      actuationTime = cellsPulseLength[i][0];
    }
    
    if (currentTime < actuationTime + lastUpdated[i]) { continue; }
    
    lastUpdated[i] = currentTime;
    
    currentAngle[i] = getSensorAngleInDegs(i);
    targetAngle[i] = target;
    cellsState[i] = (abs(angleDiffs[i]) < tolerance) ? 1 : 0;
    
    uint8_t cellLeftBagIndex = i * 2;
    uint8_t cellRightBagIndex = i * 2 + 1;
    
    if (!actuated[i] && abs(angleDiffs[i]) > tolerance) {
      actuated[i] = true;
      
      if (angleDiffs[i] < 0) {
        if (initializeInflation) { vent(cellRightBagIndex); }
        inflate(cellLeftBagIndex);
      } else {
        if (initializeInflation) { vent(cellLeftBagIndex); }
        inflate(cellRightBagIndex);
      }
      
    } else {
      lock(cellLeftBagIndex);
      lock(cellRightBagIndex);
      // wait for a period for next control (important, thus needs actuated[i] to check)
      lastUpdated[i] += actuationInverval;
      actuated[i] = false;
      
      if (abs(angleDiffs[i]) > tolerance) {
        if (angleDiffs[i] == prevAngleDiffs[i]) {
          numUnchangedAngleDiff[i] += 1;
        } else {
          numUnchangedAngleDiff[i] = 0;
        }
      }
      prevAngleDiffs[i] = angleDiffs[i];
    }
  }
  
  for (int j=start; j<end; j++) {
    currentAngle[j] = getSensorAngleInDegs(j);
    angleDiffs[j] = targetAngle[j] - currentAngle[j];
    cellsState[j] = (abs(angleDiffs[j]) < tolerance) ? 1 : 0;
  }
  
  loopCount++;
  if ((loopCount % 500) == 0) {
    for (int j=start; j<end; j++) {
      // copy the data from the console to excel for visualization
      Serial.print("CA: ");
      Serial.print(currentAngle[j]);
      Serial.print(";");
      Serial.print(" TA: ");
      Serial.print(targetAngle[j]);
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
    for (int j=start; j<end; j++) {
      // lock all airbags in case over-shooting
      uint8_t cellLeftBagIndex = j * 2;
      uint8_t cellRightBagIndex = j * 2 + 1;
      lock(cellLeftBagIndex);
      lock(cellRightBagIndex);
      Serial.print("Reached to the target angle: ");
      Serial.println(currentAngle[j]);
    }
    
    if (initializeInflation) { delay(30000); } // maintain the target angle for a while
    for (int j=start; j<end; j++) {
      // reset actuation speed
      cellsPulseLength[j][0] = constCellsPulseLength[j][0];
      cellsPulseLength[j][1] = constCellsPulseLength[j][1];
      uint8_t cellLeftBagIndex = j * 2;
      uint8_t cellRightBagIndex = j * 2 + 1;
      if (initializeInflation) {
        vent(cellLeftBagIndex);
        vent(cellRightBagIndex);
      }
    }
    if (initializeInflation) { delay(200000); } // maintain the vent state for a while and then restart
  }
}

void initializeAirbags(int start, int end) {
  // need the start index and the end index of cells
  for (int i=start; i<end; i++) {
    uint8_t cellLeftBagIndex = i * 2;
    uint8_t cellRightBagIndex = i * 2 + 1;
    
    int num = 2;
    for (int j=0; j<num; j++) {
      inflate(cellLeftBagIndex);
      delayMicroseconds(8000);
      lock(cellLeftBagIndex);
      inflate(cellRightBagIndex);
      delayMicroseconds(8000);
      lock(cellRightBagIndex);
      delay(actuationInverval/1000);
    }
  }
  
  delay(500);
  // while(!allReached) { separateUpdate(start, end, 90.0); }
  
  initializeInflation = true;
  for (int i=start; i<end; i++) { prevAngleDiffs[i] = 0; }
  Serial.println("Initialization is finished"); Serial.println();
}

void unifiedUpdate(float angleSteps[][CellNum], int stepSize, int waiting) {
  for (int i=0; i<CellNum; i++) {
    unsigned long currentTime = micros();
    // timeDiffs[i] = (long)currentTime - lastUpdated[i]; // calculate the duration of single forloop (unit is microsecond)
    
    // currentAngle[i] = getSensorAngleInDegs(i); // raw signal
    currentAngle[i] = movingAverage(i, getSensorAngleInDegs(i)); // smooth signal using moving average filter
    targetAngle[i] = angleSteps[stepIndex][i];
    angleDiffs[i] = targetAngle[i] - currentAngle[i];
    
    unsigned long actuationTime;
    if (angleDiffs[i] > 0) {
      if (numUnchangedAngleDiff[i] >= 10) { // *changed to 3
        // linear control
        // if (abs(angleDiffs[i]) > 5.0) {
        //   cellsPulseLength[i][1] = cellsPulseLength[i][1] * 1.05;
        // } else {
        //   cellsPulseLength[i][1] = cellsPulseLength[i][1] * 1.03;
        // }
        if (useProportionalControl) {
          if (abs(angleDiffs[i]) > 3.0) {
            float prop_p = (abs(angleDiffs[i]) * prop_factor + abs(currentAngle[i])) / abs(currentAngle[i]); // proportional control
            cellsPulseLength[i][1] = cellsPulseLength[i][1] * prop_p;
          } else {
            // cellsPulseLength[i][1] = cellsPulseLength[i][1] * 1.1;
            // float prop_p = (abs(angleDiffs[i]) * prop_factor + abs(currentAngle[i])) / abs(currentAngle[i]); // proportional control
            // cellsPulseLength[i][1] = cellsPulseLength[i][1] * prop_p;
            cellsPulseLength[i][1] += 50;
          }
          numUnchangedAngleDiff[i] = 0;
        } else {
          cellsPulseLength[i][1] += 200;
          numUnchangedAngleDiff[i] = 0;
        }
      }
      actuationTime = cellsPulseLength[i][1];
    } else {
      if (numUnchangedAngleDiff[i] >= 10) {
        // linear control
        // if (abs(angleDiffs[i]) > 5.0) {
        //   cellsPulseLength[i][0] = cellsPulseLength[i][0] * 1.05;
        // } else {
        //   cellsPulseLength[i][0] = cellsPulseLength[i][0] * 1.03;
        // }
        if (useProportionalControl) {
          if (abs(angleDiffs[i]) > 3.0) {
            float prop_p = (abs(angleDiffs[i]) * prop_factor + abs(currentAngle[i])) / abs(currentAngle[i]); // proportional control
            cellsPulseLength[i][0] = cellsPulseLength[i][0] * prop_p;
          } else {
            // cellsPulseLength[i][0] = cellsPulseLength[i][0] * 1.1;
            // float prop_p = (abs(angleDiffs[i]) * prop_factor + abs(currentAngle[i])) / abs(currentAngle[i]); // proportional control
            // cellsPulseLength[i][0] = cellsPulseLength[i][0] * prop_p;
            cellsPulseLength[i][0] += 50;
          }
          numUnchangedAngleDiff[i] = 0;
        } else {
          cellsPulseLength[i][0] += 200;
          numUnchangedAngleDiff[i] = 0;
        }
      }
      actuationTime = cellsPulseLength[i][0];
    }
    
    if (currentTime < actuationTime + lastUpdated[i]) { continue; }
    
    lastUpdated[i] = currentTime;
    
    uint8_t cellLeftBagIndex = i * 2;
    uint8_t cellRightBagIndex = i * 2 + 1;
    
    if (!actuated[i] && abs(angleDiffs[i]) > tolerance) {
      actuated[i] = true;
      
      if (angleDiffs[i] < 0) {
        vent(cellRightBagIndex);
        inflate(cellLeftBagIndex);
      } else {
        vent(cellLeftBagIndex);
        inflate(cellRightBagIndex);
      }
          
    } else {
      // in case vent not happened
      if (overShoot[i]) {
        lock(cellLeftBagIndex);
        lock(cellRightBagIndex);
        
      } else {
        if (angleDiffs[i] < 0) {
          lock(cellLeftBagIndex);
        } else {
          lock(cellRightBagIndex);
        }
      }
      
      // lock(cellLeftBagIndex);
      // lock(cellRightBagIndex);
      
      // wait for a period for next control (important, thus needs actuated[i] to check)
      lastUpdated[i] += actuationInverval;
      actuated[i] = false;
      
      if (abs(angleDiffs[i]) > tolerance) {
        if (abs(abs(angleDiffs[i]) - abs(prevAngleDiffs[i])) < 0.5) { // =0 or <1 or <0.5
          numUnchangedAngleDiff[i] += 1;
        } else {
          numUnchangedAngleDiff[i] = 0;  
        }
      } else {
        // *reset the actuation puleseLength in case over shooting
        cellsPulseLength[i][0] = constCellsPulseLength[i][0];
        cellsPulseLength[i][1] = constCellsPulseLength[i][1];
      }
      
      // check if overshooting happens
      if (prevAngleDiffs[i] < 0 && angleDiffs[i] > 0 || prevAngleDiffs[i] > 0 && angleDiffs[i] < 0) {
        cellsPulseLength[i][0] = constCellsPulseLength[i][0];
        cellsPulseLength[i][1] = constCellsPulseLength[i][1];
        numUnchangedAngleDiff[i] = 0;
        overShoot[i] = true;
      }
      
      prevAngleDiffs[i] = angleDiffs[i];
    }
  }
  
  for (int j=0; j<CellNum; j++) {
    // currentAngle[j] = getSensorAngleInDegs(j);
    currentAngle[j] = movingAverage(j, getSensorAngleInDegs(j));
    angleDiffs[j] = targetAngle[j] - currentAngle[j];
    cellsState[j] = (abs(angleDiffs[j]) < tolerance) ? 1 : 0;
  }
  
  loopCount++;
  if ((loopCount % 100) == 0) {
  //     for (int j=0; j<CellNum; j++) {
  //       Serial.print(j); Serial.print(" cell state: "); Serial.println(cellsState[j]);
  //       if (!cellsState[j]) {
  //         Serial.print("CA: ");
  //         Serial.print(currentAngle[j]);
  //         Serial.print(";");
  //         Serial.print(" TA: ");
  //         Serial.println(targetAngle[j]);
  //       }
  //       
  //       // Serial.print(j); Serial.print(" timeDiff "); Serial.println(timeDiffs[j]);
  //     }
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
    if (tryLimitNum == tryLimitValue) { tryLimitReached = true; }
  }
  
  allReached = true;
  for (int j=0; j<CellNum; j++) {
    if (cellsState[j] == false) {
      allReached = false;
      break;
    }
  }
  
  if (allReached) {
    for (int j=0; j<CellNum; j++) {
      // lock all airbags in case over-shooting
      uint8_t cellLeftBagIndex = j * 2;
      uint8_t cellRightBagIndex = j * 2 + 1;
      lock(cellLeftBagIndex);
      lock(cellRightBagIndex);
    }

    for (int j=0; j<CellNum; j++) {
      cellsPulseLength[j][0] = constCellsPulseLength[j][0];
      cellsPulseLength[j][1] = constCellsPulseLength[j][1];
      overShoot[j] = 0; // reset overShoot status
    }
    
    Serial.print("Point "); Serial.print(stepIndex); Serial.println(" is satisfied!"); Serial.println();
    for (int j=0; j<CellNum; j++) { Serial.print(movingAverage(j, getSensorAngleInDegs(j))); Serial.print("; "); }
    Serial.println();
    stepIndex += 1;
    tryLimitNum = 0;
    delay(waiting);
    
  } else if (tryLimitReached) {
    for (int j=0; j<CellNum; j++) {
      uint8_t cellLeftBagIndex = j * 2;
      uint8_t cellRightBagIndex = j * 2 + 1;
      lock(cellLeftBagIndex);
      lock(cellRightBagIndex);
    }
    
    for (int j=0; j<CellNum; j++) {
      cellsPulseLength[j][0] = constCellsPulseLength[j][0];
      cellsPulseLength[j][1] = constCellsPulseLength[j][1];
      overShoot[j] = 0; // reset overShoot status
    }
    
    Serial.print("The point "); Serial.print(stepIndex); Serial.println(" is passed");
    stepIndex += 1;
    tryLimitReached = false;
    tryLimitNum = 0;
    delay(waiting);
  }
  
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
      unifiedUpdate(mAngleSteps, mPointStepSize, 5000);
    }
    if (loopMovement) {
      if (updateFinished) { 
        ventAllAirbags();
        delay(500);
        updateFinished = false;
      }
    }
  } else {
    // get user input of one target point
    if (!targetRecorded) {
      while (Serial.available() == 0) {
        Serial.println("Please move the grid corner to the target point, and input 1 to store the data");
        delay(1000);
        testSensorSignal();
      }
    }
    
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
    
  } else {
    while (Serial.available() == 0) {
      if (pathPointIndex == 0) {
        Serial.println("Input 1 to start recording angles");
        delay(1000);
      }
    }
  }
  
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
  // use a high baud rate for quicker serial communication
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
  
  for (int i=0; i<CellNum; i++) {
    // adapted to 13V power supply, 20~30 psi air pressure
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
  update = 1;
  
  if (update == 1) {
    int start = 0;
    int end = 1;
    // // draw a path according to angle data from the software (flower)
    // if (!initializeInflation) {
    //   delay(1000);
    //   initializeAirbags(start, end);
    //   // delay(1000);
    // }
    initializeInflation = true;
    separateUpdate(start, end, 75.0);
    
    // if (updateFinished) {
    //   // testSensorSignal();
    // } else {
    //   unifiedUpdate(waterDropGeneratedAngles, mPathStepSize, 0);
    // }
    
  } else if (update == 2) {
    int evalCell = 0;
    testSensorSignal(evalCell);
    // testSensorSignal();
    
  } else if (update == 3) {
    ventAllAirbags();
    
  } else if (update == 4) {
    bool loopMovement = 1;
    
    // input interaction with point
    // manualInputPoint(loopMovement);
    // if (!updateFinished) {
    //   unifiedUpdate(mAngleSteps, mPointStepSize, 50000);
    // }
    // if (loopMovement) {
    //   if (updateFinished) {
    //     ventAllAirbags();
    //     delay(500);
    //     updateFinished = false;
    //     stepIndex = 0;
    //   }
    // }
    
    // input interaction with path with pathRecordingInterval as parameter
    manualInputPath(500);
    
    // if (!updateFinished) {
    //   unifiedUpdate(collectedAngles, mPathStepSize, 50);
    // }
    // if (loopMovement) {
    //   if (updateFinished) { 
    //     ventAllAirbags();
    //     delay(100);
    //     updateFinished = false;
    //     stepIndex = 0;
    //   }
    // }
    
  }
}
