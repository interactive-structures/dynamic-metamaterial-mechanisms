#ifndef CONTROLLOOP_H
#define CONTROLLOOP_H

class ControlLoop
{
private:
    int CellNum;
    int* readPin;
    int (*bagPin)[2];
    unsigned long (*constCellsPulseLength)[2];
    unsigned long (*cellsPulseLength)[2];
    int stepIndex = 0;
    int stepSize = 0;
    
    unsigned long* lastUpdated;
    long* timeDiffs;
    bool* actuated;
    bool* cellsState;
    float* angleDiffs;
    float* prevAngleDiffs;
    float* targetAngle;
    float* currentAngle;
    uint8_t numUnchanged = 0;
    uint8_t* numUnchangedAngleDiff;
    
    bool update;
    bool allReached;
    uint8_t tryLimitNum;
    bool tryLimitReached; // give a limit number in case never satisfied
    
    bool initializeInflation = false;
    const float tolerance = 1.0;
    unsigned long actuationInverval = 200000;
    unsigned int loopCount = 0;
    
public:
    float** angleSteps;
    ControlLoop(int CellNum, int* readPin, int (*bagPin)[2], unsigned long (*constCellsPulseLength)[2], unsigned long (*cellsPulseLength)[2], int stepSize);
    ~ControlLoop();

    float getSensorAngleInDegs(int sensorIndex);
    void testSensorSignal();
    void inflate(int index);
    void lock(int index);
    void vent(int index);
    void ventAllAirbags();
    
    void separateUpdate(int start, int end, float target);
    void initializeAirbags(int start, int end);
    void unifiedUpdate();
    
    void setup();
    void loop();
};

ControlLoop::ControlLoop(int CellNum, int* readPin, int (*bagPin)[2], unsigned long (*constCellsPulseLength)[2], unsigned long (*cellsPulseLength)[2], int stepSize) {
    this->CellNum = CellNum;
    
    this->readPin = new int[CellNum];
    for (int i=0; i<CellNum; ++i) {
        this->readPin[i] = readPin[i];
    }
    
    this->bagPin = new int[CellNum*2][2];
    for (int i=0; i<CellNum*2; ++i) {
        this->bagPin[i][0] = bagPin[i][0];
        this->bagPin[i][1] = bagPin[i][1];
    }
    
    this->constCellsPulseLength = new unsigned long[CellNum][2];
    memcpy(this->constCellsPulseLength, constCellsPulseLength, sizeof(unsigned long)*CellNum*2);
    
    this->cellsPulseLength = new unsigned long[CellNum][2];
    memcpy(this->cellsPulseLength, cellsPulseLength, sizeof(unsigned long)*CellNum*2);
    
    this->stepSize = stepSize;
    
    // allocate dynamic memory
    angleSteps = new float*[stepSize];
    for (int i=0; i<stepSize; ++i) {
        angleSteps[i] = new float[CellNum];
    }
    
    lastUpdated = new unsigned long[CellNum];
    timeDiffs = new long[CellNum];
    actuated = new bool[CellNum];
    cellsState = new bool[CellNum];
    angleDiffs = new float[CellNum];
    prevAngleDiffs = new float[CellNum];
    targetAngle = new float[CellNum];
    currentAngle = new float[CellNum];
    numUnchangedAngleDiff = new uint8_t[CellNum];
}

ControlLoop::~ControlLoop() {
  for (int i=0; i<stepSize; ++i) {
    delete[] angleSteps[i];
  }
  delete[] angleSteps;
}

inline float ControlLoop::getSensorAngleInDegs(int sensorIndex) {
    // CJMCU-103 Rotary Angle Module SV01A103AEA01R00 has effective rotation angle 333.3 which can guarantee linearity
    // 512 indicates 90 degs
    float incomingByte = analogRead(readPin[sensorIndex]);
    return (incomingByte / 1024) * 333.3 - 76.65;
}

inline void ControlLoop::testSensorSignal() {
    for (int i=0; i<CellNum; i++) {
        Serial.print("Sensor "); Serial.print(i); Serial.print(": ");
        Serial.println(getSensorAngleInDegs(i));
        delay(500);
    }
    Serial.println();
}

inline void ControlLoop::inflate(int index) {
    digitalWrite(bagPin[index][0], LOW);
    digitalWrite(bagPin[index][1], HIGH);
}

inline void ControlLoop::lock(int index) {
    digitalWrite(bagPin[index][0], LOW);
    digitalWrite(bagPin[index][1], LOW);
}

inline void ControlLoop::vent(int index) {
    digitalWrite(bagPin[index][0], HIGH);
    digitalWrite(bagPin[index][1], LOW);
}

inline void ControlLoop::ventAllAirbags() {
    for (int i=0; i<CellNum; i++) {
        int cellLeftBagIndex = i * 2;
        int cellRightBagIndex = i * 2 + 1;
        vent(cellLeftBagIndex);
        vent(cellRightBagIndex);
    }
}

void ControlLoop::separateUpdate(int start, int end, float target) {
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
    angleDiffs[i] = targetAngle[i] - currentAngle[i];
    
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
    
    Serial.print("currentAngle:");
    Serial.println(currentAngle[j]);
    Serial.print("targetAngle:");
    Serial.println(targetAngle[j]);
    
    // Serial.print(j); Serial.print(" timeDiff "); Serial.println(timeDiffs[j]);
    
    // Serial.print("Diff");
    // Serial.print(j);
    // Serial.print(":");
    // Serial.println(angleDiffs[j]);
  }
  Serial.println();
  
  allReached = true;
  for (int j=start; j<end; j++) {
    if (cellsState[j] == false) {
      allReached = false;
      break;
    }
  }
  
  if (allReached) {
    for (int j=start; j<end; j++) {
      // 可能是之前allReach之后没有及时锁住会导致角度偏移不准确？
      uint8_t cellLeftBagIndex = j * 2;
      uint8_t cellRightBagIndex = j * 2 + 1;
      lock(cellLeftBagIndex);
      lock(cellRightBagIndex);
    }
    delay(10000);
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
    delay(3000);
  }
}

void ControlLoop::initializeAirbags(int start, int end) {
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
  
  delay(2000);
  while(!allReached) { separateUpdate(start, end, 90.0); }
  
  initializeInflation = true;
  Serial.println("Initialization is finished"); Serial.println();
}

void ControlLoop::unifiedUpdate() {
  for (int i=0; i<CellNum; i++) {
    unsigned long currentTime = micros();
    // timeDiffs[i] = (long)currentTime - lastUpdated[i]; // calculate the duration of single forloop (unit is microsecond)
    
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
  
  for (int j=0; j<CellNum; j++) {
    currentAngle[j] = getSensorAngleInDegs(j);
    angleDiffs[j] = targetAngle[j] - currentAngle[j];
    cellsState[j] = (abs(angleDiffs[j]) < tolerance) ? 1 : 0;
  }
  
  loopCount++;
  if ((loopCount % 1000) == 0) {
    for (int j=0; j<CellNum; j++) {
      Serial.print(j); Serial.print(" cell state "); Serial.println(cellsState[j]);
      if (!cellsState[j]) {
        Serial.print("CA: ");
        Serial.println(currentAngle[j]);
        Serial.print("TA: ");
        Serial.println(targetAngle[j]);
      }
      // Serial.print(j); Serial.print(" timeDiff "); Serial.println(timeDiffs[j]);
    }
    
    // for (int k=0; k < CellNum; k++) {
    //   Serial.print("Diff");
    //   Serial.print(k);
    //   Serial.print(":");
    //   Serial.println(angleDiffs[k]);
    // }
    
    Serial.println();
    loopCount = 0;
    tryLimitNum += 1;
    if (tryLimitNum == 100) { tryLimitReached = true; }
  }
  
  allReached = true;
  for (int j=0; j<CellNum; j++) {
    if (cellsState[j] == false) {
      allReached = false;
      break;
    }
  }
  
  if (allReached) {
    Serial.print("Point "); Serial.print(stepIndex); Serial.println(" is satisfied!"); Serial.println();
    stepIndex += 1;
    for (int j=0; j<CellNum; j++) {
      // Serial.println(angleDiffs[j]);
      uint8_t cellLeftBagIndex = j * 2;
      uint8_t cellRightBagIndex = j * 2 + 1;
      lock(cellLeftBagIndex);
      lock(cellRightBagIndex);
      cellsPulseLength[j][0] = constCellsPulseLength[j][0];
      cellsPulseLength[j][1] = constCellsPulseLength[j][1];
    }
    tryLimitNum = 0;
    delay(5000);
  } else if (tryLimitReached) {
    Serial.print("The point "); Serial.print(stepIndex); Serial.println(" is passed");
    stepIndex += 1;
    for (int j=0; j<CellNum; j++) {
      uint8_t cellLeftBagIndex = j * 2;
      uint8_t cellRightBagIndex = j * 2 + 1;
      lock(cellLeftBagIndex);
      lock(cellRightBagIndex);
      cellsPulseLength[j][0] = constCellsPulseLength[j][0];
      cellsPulseLength[j][1] = constCellsPulseLength[j][1];
    }
    tryLimitReached = false;
    tryLimitNum = 0;
    delay(5000);
  }
  
  if (stepIndex == stepSize) {
    Serial.println("All points have been reached!");
    exit(0);
  }
}

void ControlLoop::setup() {
    // set sensor signal pin as INPUT
    for (int i=0; i<CellNum; i++) {
        pinMode(readPin[i], INPUT);
    }
    
    // set valve control pin as OUTPUT
    for (int i=0; i<CellNum * 2; i++) {
        pinMode(bagPin[i][0], OUTPUT);
        digitalWrite(bagPin[i][0], LOW);
        pinMode(bagPin[i][1], OUTPUT);
        digitalWrite(bagPin[i][1], LOW);
    }
    
    delay(100);
}

void ControlLoop::loop() {
  int start = 5;
  int end = 6;
  update = 1;
  if (update) {
    if (!initializeInflation) {
      initializeAirbags(start, end);
      delay(2000);
    }
    float target = 105.0;
    separateUpdate(start, end, target);
    // unifiedUpdate();
  } else {
    // testSensorSignal();
    ventAllAirbags();
  }
}

#endif CONTROLLOOP_H