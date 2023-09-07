#ifndef CONTROLLOOP_H
#define CONTROLLOOP_H

// number of active cells
const int CellNum = 6;

class ControlLoop
{
private:
    int readPin[CellNum];
    int bagPin[CellNum*2][2];
    unsigned long constCellsPulseLength[CellNum][2];
    unsigned long cellsPulseLength[CellNum][2];
    int stepIndex = 0;
    int stepSize = 0;
    
    unsigned long lastUpdated[CellNum];
    long timeDiffs[CellNum];
    bool actuated[CellNum];
    bool cellsState[CellNum];
    float angleDiffs[CellNum];
    float prevAngleDiffs[CellNum];
    float targetAngle[CellNum];
    float currentAngle[CellNum];
    uint8_t numUnchangedAngleDiff[CellNum];
    uint8_t numUnchanged = 0;
    
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
    ControlLoop(int readPin[CellNum], int bagPin[CellNum*2][2], unsigned long constCellsPulseLength[CellNum][2], unsigned long cellsPulseLength[CellNum][2], int stepSize);
    ~ControlLoop();

    float getSensorAngleInDegs(int sensorIndex);
    void testSensorSignal();
    void testSensorSignal(int sensorIndex);
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

#include "ControlLoop.h"

ControlLoop::ControlLoop(int readPin[CellNum], int bagPin[CellNum*2][2], unsigned long constCellsPulseLength[CellNum][2], unsigned long cellsPulseLength[CellNum][2], int stepSize) {
  for (int i=0; i<CellNum; ++i) {
    this->readPin[i] = readPin[i];
  }
  
  for (int i=0; i<CellNum*2; ++i) {
    this->bagPin[i][0] = bagPin[i][0];
    this->bagPin[i][1] = bagPin[i][1];
  }
  
  for (int i=0; i<CellNum; ++i) {
    this->constCellsPulseLength[i][0] = constCellsPulseLength[i][0];
    this->constCellsPulseLength[i][1] = constCellsPulseLength[i][1];
  }
  
  for (int i=0; i<CellNum; ++i) {
    this->cellsPulseLength[i][0] = cellsPulseLength[i][0];
    this->cellsPulseLength[i][1] = cellsPulseLength[i][1];
  }
  
  this->stepSize = stepSize;
  
  // allocate dynamic memory
  angleSteps = new float*[stepSize];
  for (int i=0; i<stepSize; ++i) {
      angleSteps[i] = new float[CellNum];
  }
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

inline void ControlLoop::testSensorSignal(int sensorIndex) {
  Serial.print("Sensor "); Serial.print(sensorIndex); Serial.print(": ");
  // float incomingByte = analogRead(readPin[sensorIndex]) - 31;
  // float calibrateAngle = (incomingByte / 1024) * 333.3 - 76.65;
  Serial.println(getSensorAngleInDegs(sensorIndex));
  delay(500);
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
    Serial.println(currentAngle[i]);
    targetAngle[i] = target;
    // Serial.println(&targetAngle[i]);
    // angleDiffs[i] = targetAngle[i] - currentAngle[i];
    Serial.println(currentAngle[i]);
    Serial.println(target);
    // Serial.println(targetAngle[i] - currentAngle[i]);
    // Serial.println(angleDiffs[i]);
    
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
  if ((loopCount % 1000) == 0) {
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
    Serial.println("allReached for test");
    for (int j=start; j<end; j++) {
      // lock all airbags in case over-shooting
      uint8_t cellLeftBagIndex = j * 2;
      uint8_t cellRightBagIndex = j * 2 + 1;
      lock(cellLeftBagIndex);
      lock(cellRightBagIndex);
    }
    
    if (initializeInflation) { delay(10000); } // maintain the target angle for a while
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
    if (initializeInflation) { delay(5000); } // maintain the vent state for a while and then restart
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
  
  delay(500);
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
  int evalCell = 2;
  int start = 2;
  int end = 3;
  update = 0;
  if (update) {
    if (!initializeInflation) {
      initializeAirbags(start, end);
      delay(2000);
    }
    // float target = 105.0;
    // separateUpdate(start, end, target);
    // unifiedUpdate();
  } else {
    testSensorSignal(evalCell);
    // ventAllAirbags();
  }
}

#endif CONTROLLOOP_H