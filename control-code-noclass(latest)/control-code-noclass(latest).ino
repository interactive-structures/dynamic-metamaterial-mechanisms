// Setup-3
const int CellNum = 6;
int readPin[CellNum] = {A6,A4,A3,A2,A1,A0};
int bagPin[CellNum*2][2] = {
  {3,2}, {5,4}, //1
  {40,41}, {42,43}, //2
  {44,45}, {46,47}, //3
  {28,31}, {32,33}, //4
  {34,35}, {36,37}, //5
  {7,6}, {9,8} //6
};

// Setup-1
// const int CellNum = 2;
// int readPin[CellNum] = {A0, A3};
// int bagPin[CellNum*2][2] = {{6,7},{8,9},{2,3},{4,5}};
// Setup-2
// const int CellNum = 4;
// int readPin[CellNum] = {A0, A1, A2, A3};
// int bagPin[CellNum*2][2] = {
//   {40,41},{42,43},
//   {44,45},{46,48},
//   {28,31},{32,33},
//   {34,35},{36,37}
// };

const float tolerance = 1.5; // adjust this for application
bool initializeInflation = false;
unsigned long constCellsPulseLength[CellNum][2];
unsigned long cellsPulseLength[CellNum][2];
int minActuation = 4500; // adjust this for application
unsigned long actuationInverval = 200000;
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
uint8_t update;
bool allReached;
bool updateFinished;
uint8_t tryLimitNum;
bool tryLimitReached; // give a limit tries in case never satisfied
int stepIndex = 8;
const int pathStepSize = 99; // number of points on the path defined in software

// user input interaction
bool targetRecorded;
bool startReplay;
unsigned long lastRecordingTime;
unsigned long pathRecordingInterval = 1000;
uint8_t pathPointIndex = 0;
const int mPointStepSize = 1;
float mAngleSteps[mPointStepSize][CellNum] = {{86.42, 99.76, 94.56, 101.72, 93.91, 90.33}};
const int mPathStepSize = 17; // number of sampling points on a user-defined path

float mPathAngleSteps[mPathStepSize][CellNum] = {
  {89.67, 89.35, 82.19, 85.12, 94.80, 96.25}, //0
  {85.12, 99.44, 94.23, 102.37, 93.58, 93.25}, //1
  {80.56, 103.02, 103.02, 112.46, 101.39, 89.02}, //2
  {75.98, 100.23, 98.46, 117.02, 103.02, 91.60}, //3
  {73.07, 97.81, 100.74, 97.76, 97.49, 88.65}, //4
  {78.61, 88.05, 92.28, 90.98, 91.30, 90.33}, //5
  {83.56, 89.35, 94.18, 69.19, 85.61, 93.91}, //6
  {83.14, 84.07, 92.21, 66.52, 82.61, 100.84}, //7
  {89.00, 81.02, 81.15, 68.98, 70.40, 101.42}, //8
  {90.98, 87.14, 77.68, 75.77, 78.54, 99.49}, //9
  {100.09, 82.84, 81.73, 65.01, 66.70, 98.44}, //10
  {100.42, 82.61, 83.03, 58.38, 60.31, 101.44}, //11
  {100.74, 90.12, 88.42, 73.12, 63.54, 101.51}, //12
  {96.98, 87.07, 86.16, 85.12, 86.98, 96.93}, //13
  {96.18, 91.30, 94.72, 99.56, 108.21, 89.21}, //14
  {88.35, 91.05, 96.51, 103.93, 107.49, 90.00}, //15
  {86.42, 89.35, 83.49, 89.02, 94.56, 93.58} //16
};

float getSensorAngleInDegs(int sensorIndex) {
  // CJMCU-103 Rotary Angle Module SV01A103AEA01R00 has effective rotation angle 333.3 which can guarantee linearity
  // 512 indicates 90 degs
  
  // float incomingByte = analogRead(readPin[sensorIndex]) + 45;
  // return (incomingByte / 1024) * 333.3 - 76.65;
  
  float incomingByte = analogRead(readPin[sensorIndex]);
  return 256.65 - (incomingByte / 1024) * 333.3;
}

void testSensorSignal() {
  for (int i=0; i<CellNum; i++) {
    Serial.print("Sensor "); Serial.print(i); Serial.print(": ");
    Serial.println(getSensorAngleInDegs(i));
    delay(500);
  }
  Serial.println();
}

void testSensorSignal(int sensorIndex) {
  Serial.print("Sensor "); Serial.print(sensorIndex); Serial.print(": ");
  float incomingByte = analogRead(readPin[sensorIndex]);
  // float calibrateAngle = (incomingByte / 1024) * 333.3 - 76.65;
  float calibrateAngle = 256.65 - (incomingByte / 1024) * 333.3;
  Serial.println(calibrateAngle);
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
        cellsPulseLength[i][1] += 200;
        numUnchangedAngleDiff[i] = 0;
      }
      actuationTime = cellsPulseLength[i][1];
      
    } else {
      if (numUnchangedAngleDiff[i] >= 2) {
        cellsPulseLength[i][0] += 200;
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
    if (initializeInflation) { delay(20000); } // maintain the vent state for a while and then restart
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
  while(!allReached) { separateUpdate(start, end, 90.0); }
  
  initializeInflation = true;
  for (int i=start; i<end; i++) { prevAngleDiffs[i] = 0; }
  Serial.println("Initialization is finished"); Serial.println();
}

void unifiedUpdate(float angleSteps[][CellNum], int stepSize, int waiting) {
  for (int i=0; i<CellNum; i++) {
    unsigned long currentTime = micros();
    // timeDiffs[i] = (long)currentTime - lastUpdated[i]; // calculate the duration of single forloop (unit is microsecond)
    
    unsigned long actuationTime;
    if (angleDiffs[i] > 0) {
      if (numUnchangedAngleDiff[i] >= 3) { // *changed to 3
        cellsPulseLength[i][1] += 200;
        numUnchangedAngleDiff[i] = 0;
      }
      actuationTime = cellsPulseLength[i][1];
    } else {
      if (numUnchangedAngleDiff[i] >= 3) {
        cellsPulseLength[i][0] += 200;
        numUnchangedAngleDiff[i] = 0;
      }
      actuationTime = cellsPulseLength[i][0];
    }
    
    if (currentTime < actuationTime + lastUpdated[i]) { continue; }
    
    lastUpdated[i] = currentTime;
    
    currentAngle[i] = getSensorAngleInDegs(i);
    targetAngle[i] = angleSteps[stepIndex][i];
    angleDiffs[i] = targetAngle[i] - currentAngle[i];
    
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
      if (angleDiffs[i] < 0) { 
        lock(cellLeftBagIndex); 
      } else {
        lock(cellRightBagIndex);
      }
      // wait for a period for next control (important, thus needs actuated[i] to check)
      lastUpdated[i] += actuationInverval;
      actuated[i] = false;
      
      if (abs(angleDiffs[i]) > tolerance) {
        if (angleDiffs[i] == prevAngleDiffs[i]) {
          numUnchangedAngleDiff[i] += 1;
        } else {
          numUnchangedAngleDiff[i] = 0;
          // *reset the actuation puleseLength
          cellsPulseLength[i][0] = constCellsPulseLength[i][0];
          cellsPulseLength[i][1] = constCellsPulseLength[i][1];
        }
      }
      prevAngleDiffs[i] = angleDiffs[i];
    }
  }
  
  for (int j=0; j<CellNum; j++) {
    currentAngle[j] = getSensorAngleInDegs(j);
    angleDiffs[j] = targetAngle[j] - currentAngle[j];
    cellsState[j] = (abs(angleDiffs[j]) < tolerance) ? 1 : 0;
  }
  
  loopCount++;
  if ((loopCount % 1000) == 0) {
    for (int j=0; j<CellNum; j++) {
      // Serial.print(j); Serial.print(" cell state: "); Serial.println(cellsState[j]);
      // if (!cellsState[j]) {
      Serial.print("CA: ");
      Serial.print(currentAngle[j]);
      Serial.print(";");
      Serial.print(" TA: ");
      Serial.println(targetAngle[j]);
      
      // Serial.println(angleDiffs[j]);
      // }
      // Serial.println();
      // Serial.print(j); Serial.print(" timeDiff "); Serial.println(timeDiffs[j]);
    }
    
    loopCount = 0;
    tryLimitNum += 1;
    if (tryLimitNum == 50) { tryLimitReached = true; }
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
      cellsPulseLength[j][0] = constCellsPulseLength[j][0];
      cellsPulseLength[j][1] = constCellsPulseLength[j][1];
    }
    Serial.print("Point "); Serial.print(stepIndex); Serial.println(" is satisfied!"); Serial.println();
    for (int j=0; j<CellNum; j++) {
      Serial.print(getSensorAngleInDegs(j)); Serial.print("; ");
    }
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
      cellsPulseLength[j][0] = constCellsPulseLength[j][0];
      cellsPulseLength[j][1] = constCellsPulseLength[j][1];
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
    exit(0);
  }
}

void manualInputPoint(bool loopMovement) {
  if (startReplay) {
    // start reach to the target point
    if (!updateFinished) {
      unifiedUpdate(mAngleSteps, mPointStepSize, 500);
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

void manualInputPath() {
  if (targetRecorded) {
    unsigned long currentTime = millis();
    if (currentTime - lastRecordingTime > pathRecordingInterval) {
      lastRecordingTime = currentTime;
      pathPointIndex++;
      Serial.print("P"); Serial.print(pathPointIndex); Serial.print(": ");
      for (int i=0; i<CellNum; i++) {
        Serial.print(getSensorAngleInDegs(i)); Serial.print("; ");
      }
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
    numUnchangedAngleDiff[i] = 0;
  }
  
  delay(100);
}

void loop() {
  update = 1;
  
  if (update == 1) {
    // int start = 3;
    // int end = 4;
    // draw a path according to angle data from the software (flower)
    // if (!initializeInflation) {
    //   delay(1000);
    //   initializeAirbags(start, end);
    //   delay(1000);
    // }
    // initializeInflation = true;
    // separateUpdate(start, end, 70.0);
    
    unifiedUpdate(mPathAngleSteps, mPathStepSize, 500);
    
  } else if (update == 2) {
    // int evalCell = 5;
    // testSensorSignal(evalCell);
    testSensorSignal();
    
  } else if (update == 3) {
    ventAllAirbags();
    
  } else if (update == 4) {
    bool loopMovement = 1;
    
    // input interaction with point
    manualInputPoint(loopMovement);
    // if (!updateFinished) {
    //   unifiedUpdate(mAngleSteps, mPointStepSize, 500);
    // }
    // if (loopMovement) {
    //   if (updateFinished) {
    //     ventAllAirbags();
    //     delay(500);
    //     updateFinished = false;
    //     stepIndex = 0;
    //   }
    // }
    
    // input interaction with path
    // manualInputPath();
    
    // if (!updateFinished) {
    //   unifiedUpdate(mPathAngleSteps, mPathStepSize, 10000);
    // }
    // if (loopMovement) {
    //   if (updateFinished) { 
    //     ventAllAirbags();
    //     delay(50000);
    //     updateFinished = false;
    //     stepIndex = 0;
    //   }
    // }
  }
}
