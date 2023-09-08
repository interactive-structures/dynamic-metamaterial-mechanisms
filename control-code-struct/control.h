#ifndef MOCK_VERSION
#define MOCK_VERSION 1
#endif 


#if MOCK_VERSION
	#include "Mock.h"
#endif 

#include "Input.h"


struct Cell {
	int index = -1;

	// pins that the valves for the bags are connected to
	int bagPins[2][2] = { {-1, -1},{-1, -1} }; // {{left in, left out}, {right in, right out}}
	int sensorPin = -1;


	// globally defined variables
	float tolerance = 0.0f;
	int actuationTime = -1;


	// data for control loop 
	float targetAngle = 0.0f;
	float currentAngle = 0.0f;
	float angleDiff = 0.0f;
	bool hasReached = false;
};


Cell cells[numCells];

// internal data for control loop
int angleIndex = 0;	//~~stepIndex
int numTrials = -1;
bool allReached = false;



void inflate(Cell cell, int bagIndex) {
	digitalWrite(cell.bagPins[bagIndex][BAG_IN], HIGH);
	digitalWrite(cell.bagPins[bagIndex][BAG_OUT], LOW);
}

void vent(Cell cell, int bagIndex) {
	digitalWrite(cell.bagPins[bagIndex][BAG_IN], LOW);
	digitalWrite(cell.bagPins[bagIndex][BAG_OUT], HIGH);
}

void lockBag(Cell cell, int bagIndex) {
	digitalWrite(cell.bagPins[bagIndex][BAG_IN], LOW);
	digitalWrite(cell.bagPins[bagIndex][BAG_OUT], LOW);
}

void lockCell(Cell cell) {
	lockBag(cell, BAG_LEFT);
	lockBag(cell, BAG_RIGHT);
}

void ventAllAirbags() {
	for (int i = 0; i < numCells; i++) {
		vent(cells[i], BAG_LEFT);
		vent(cells[i], BAG_RIGHT);
	}
}

float getSensorAngleInDegs(Cell cell) {
	// CJMCU-103 Rotary Angle Module SV01A103AEA01R00 has effective rotation angle 333.3 which can guarantee linearity
	// 512 indicates 90 degs

	// float incomingByte = analogRead(readPin[sensorIndex]) + 45;
	// return (incomingByte / 1024) * 333.3 - 76.65;

	float incomingByte = analogRead(cell.sensorPin);
	return 256.65 - (incomingByte / 1024) * 333.3;
}

void initializeCells() {
	for (int i = 0; i < numCells; i++) {
		struct Cell cell;
		cell.index = i;
		cell.sensorPin = sensorPins[i];
		cell.tolerance = gTolerance;
		cell.actuationTime = gMinInflateDuration;
		cell.targetAngle = pathAngles[i][0];

		for (int j = 0; j < 2; j++)
			for (int k = 0; k < 2; k++)
				cell.bagPins[j][k] = bagPins[i][j][k];

		cells[i] = cell;
	}
}

void setProportionalActuationTime(Cell& cell) {
	const float maxAngle = 90 - 40; //maximum angle taht any given cell can get to (calibrate)
	const float scaleProportional = 0.25;

	cell.actuationTime = (cell.angleDiff / maxAngle) * scaleProportional;
	if (cell.actuationTime < gMinInflateDuration)
		cell.actuationTime = gMinInflateDuration;
}

void actuateCell(Cell& cell) {
	setProportionalActuationTime(cell);

	if (cell.angleDiff > 0) { // inflate left bag
		vent(cell, BAG_RIGHT);
		inflate(cell, BAG_LEFT);
		printf("inflate LEFT bag for %d ms", cell.actuationTime);
	}
	else {
		vent(cell, BAG_LEFT);
		inflate(cell, BAG_RIGHT);
		printf("inflate RIGHT bag for %d ms", cell.actuationTime);
	}
}

void updateCell(Cell& cell) {
	printf("\n  CELL %i: ", cell.index);

	// check if currently actuated bags are done


	// moved on to new angle on path, reset cell variables
	if (numTrials == 0) {
		//cell.hasReached = false;
		cell.targetAngle = pathAngles[cell.index][angleIndex];
	}

	// compute current angles
#if MOCK_VERSION
	cell.currentAngle = getSensorAngleInDegs(cell.targetAngle, (1.0 * gMaxTrials - numTrials) / (gMaxTrials * 1.0));
#else
	cell.currentAngle = getSensorAngleInDegs(cell);
#endif
	

	cell.angleDiff = cell.targetAngle - cell.currentAngle;
	printf("currentAngle: %f, targetAngle: %f, angleDiff: %f // ", cell.currentAngle, cell.targetAngle, cell.angleDiff);

	// check if target is reached, exit function if reached
	if (abs(cell.angleDiff) < cell.tolerance) {
		lockCell(cell);
		cell.hasReached = true;
		printf("target reached, cell locked");
		return;
	}

	// target is not reached, need to actuate cell
	cell.hasReached = false;
	actuateCell(cell);
}


void unifiedUpdate() {
	//unsigned long currentTime = micros();

	//printf("---\n ALL CELLS have reached? %s \n--- \n", allReached ? "TRUE" : "false");

	if (allReached) { //if all cells reached their angles 
		if (angleIndex >= numPathAngles) { //if this was the last angle
			printf("\n ALL DONE \n");
			exit(1);
		}
		else {
			angleIndex++;
			numTrials = 0;
			allReached = false;
			printf("\nALL REACHED. Next angle index: %d \n\n", angleIndex);
		}
	}
	else {
		if (numTrials >= gMaxTrials) //not fully reached, but moving on
		{
			allReached = true;
			printf("\nangles NOT all reached, but max trails. Next angle index: %d \n\n", angleIndex);

		}
		else {
			numTrials++;
			printf("\nNext trial: %d", numTrials);
			
			allReached = true;

			for (int ci = 0; ci < numCells; ci++) {
				updateCell(cells[ci]);
				printf(" // cells[%d].hasReached: %d", ci, cells[ci].hasReached);

				if (allReached) {
					allReached = cells[ci].hasReached;
				}
			}
		}
	}
}

void setup() {
	//// use a high baud rate for quicker serial communication
	//Serial.begin(2000000);

	// set all sensor signal pins as INPUT
	for (int i = 0; i < numCells; i++) {
		pinMode(cells[i].sensorPin, INPUT);
	}

	// set pins for all valves
	for (int i = 0; i < numCells; i++) {
		int pin = cells[i].bagPins[BAG_LEFT][BAG_IN];
		pinMode(pin, OUTPUT);
		digitalWrite(pin, LOW);

		pin = cells[i].bagPins[BAG_LEFT][BAG_OUT];
		pinMode(pin, OUTPUT);
		digitalWrite(pin, LOW);

		pin = cells[i].bagPins[BAG_RIGHT][BAG_OUT];
		pinMode(pin, OUTPUT);
		digitalWrite(pin, LOW);

		pin = cells[i].bagPins[BAG_RIGHT][BAG_OUT];
		pinMode(pin, OUTPUT);
		digitalWrite(pin, LOW);
	}

	initializeCells();

	delay(100);
}

void loop() {
	unifiedUpdate();
}


#if MOCK_VERSION
int main() {
	setup();

	while (true) {
		loop();
	}

	return 0;
}
#endif 