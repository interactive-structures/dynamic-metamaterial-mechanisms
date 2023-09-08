#pragma once
#ifndef MOCK_H
#define MOCK_H

#define A0 0
#define A1 1
#define A2 2
#define A3 3
#define A4 4
#define A6 6

#define LOW 0
#define HIGH 1
#define INPUT 1
#define OUTPUT 2

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include "Input.h"

const int spread = 0.2 * 180;
//srand(time(NULL));


void digitalWrite(int pin, int value) { }
void pinMode(int pin, int value) { }
void delay(int milliseconds) { }
float analogRead(int pin) { return 0.0f; }
float abs(float x) { return fabsf(x); }

float getSensorAngleInDegs(int cellIndex, int angleIndex, int trials) {
	float targetAngle = pathAngles[cellIndex][angleIndex];
	int r = rand() % spread;
	float scaled = (1.0*gMaxTrials - trials) / (gMaxTrials*1.0);
	r = scaled * r;
	r -= (scaled*spread) / 2;

	return targetAngle + r;
}

//void generateMockOutputAngles(float targetAngles[numPathAngles][numCells], float outputAngles[numPathAngles][numCells], int numPathAngles, int numCells) {
float* generateMockOutputAngles(float targetAngles[numPathAngles][numCells]) {
	const int spread = 0.2 * 180;
	srand(time(NULL));

	float outputAngles[numPathAngles][numCells];
	for (int ci = 0; ci < numCells; ci++) {
		for (int ai = 0; ai < numPathAngles; ai++) {
			int r = rand() % spread;
			r -= spread / 2;

			outputAngles[ci][ai] = (targetAngles[ci][ai]) + r;
			printf("target/output: "); 
			printf("% .2f -->", targetAngles[ci][ai]);
			printf("% .2f \n", outputAngles[ci][ai]);
		}
	}


	return *outputAngles;
}

//float* outputAngles = generateMockOutputAngles(pathAngles);




#endif