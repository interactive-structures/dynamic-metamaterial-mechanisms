#include "Mock.h"
#include "Input.h"

#define BAG_IN 0
#define BAG_OUT 1
#define BAG_LEFT 0
#define BAG_RIGHT 1


struct Cell {
	int index;

	// pins that the valves for the bags are connected to
	int bagPins[2][2] = { {0, 0},{0, 0} }; // {{left in, left out}, {right in, right out}}
	int sensorPin;
	// globally defined variables
	float tolerance;
	int inflationDuration;


	float targetAngle;
	float currentAngle;
	bool hasReached = false;


	// data for control loop 

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

void lock(Cell cell, int bagIndex) {
	digitalWrite(cell.bagPins[bagIndex][BAG_IN], LOW);
	digitalWrite(cell.bagPins[bagIndex][BAG_OUT], LOW);
}

void vent(Cell cell, int bagIndex) {
	digitalWrite(cell.bagPins[bagIndex][BAG_IN], LOW);
	digitalWrite(cell.bagPins[bagIndex][BAG_OUT], HIGH);
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
		cell.inflationDuration = gMinInflateDuration;
		cell.targetAngle = pathAngles[i][0];
		
		for (int j = 0; j < 2; j++)
			for (int k = 0; k < 2; k++)
				cell.bagPins[j][k] = bagPins[i][j][k];
		
		cells[i] = cell;
	}
}

void updateCell(Cell &cell) {
	printf("CELL %i: \n", cell.index);

	if (numTrials == 0) { // moved on to new angle on path
		//cell.hasReached = false;
		cell.targetAngle = pathAngles[cell.index][angleIndex];
	}

	cell.currentAngle = getSensorAngleInDegs(cell.index, angleIndex, numTrials);
	float angleDiff = cell.targetAngle - cell.currentAngle;
	printf("   currentAngle: %f, targetAngle: %f, angleDiff: %f \n", cell.currentAngle, cell.targetAngle, angleDiff);

	if (abs(angleDiff) < cell.tolerance) { // target is reached
		lock(cell, BAG_LEFT);
		lock(cell, BAG_RIGHT);
		cell.hasReached = true;
		printf("   target reached, cell locked \n");
	}
	else {
		cell.hasReached = false;

		if (angleDiff > 0) { // inflate left bag
			inflate(cell, BAG_LEFT);
			delay(cell.inflationDuration);
			lock(cell, BAG_LEFT);
			printf("   inflated left bag \n");
		}
		else {
			inflate(cell, BAG_RIGHT);
			delay(cell.inflationDuration);
			lock(cell, BAG_RIGHT);
			printf("   inflated right bag \n");
		}
	}
}


void unifiedUpdate() {
	//unsigned long currentTime = micros();

	printf("---\n ALL CELLS have reached? %s \n--- \n", allReached ? "TRUE" : "false");

	if (allReached) { //if all cells reached their angles 
		if (angleIndex >= numPathAngles) { //if this was the last angle
			printf("\n ALL DONE \n");
			return;
		}
		else {
			angleIndex++;
			numTrials = 0;
			allReached = false;
			printf("Next angle index: %d \n\n", angleIndex);
		}
	}
	else {
		if (numTrials >= gMaxTrials) //not fully reached, but moving on
		{
			allReached = true;
		}
		else {
			numTrials++;
			allReached = true;

			for (int ci = 0; ci < numCells; ci++) {
				updateCell(cells[ci]);
				printf("cells[%d].hasReached: %d \n", ci, cells[ci].hasReached);

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

int main() {
	setup();

	while (true) {
		loop();
	}

	return 0;
}