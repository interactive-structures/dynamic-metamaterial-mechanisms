#pragma once
#ifndef INPUT_H
#define INPUT_H


// global parameters

#define BAG_IN 0
#define BAG_OUT 1
#define BAG_LEFT 0
#define BAG_RIGHT 1

float gTolerance = 1.5f; // [deg]; tolerance for considering target angle reached
int gMinInflateDuration = 4500; // [ms]; how long the bag should be inflated at the minimum
int gMaxTrials = 20; // max. loops to try to reach target angle
int gDwell = 500; //[ms]; a delay between each angle reached by all cells


// current cell setup (adapt per application)

const int numCells = 6;		//~~CellNum
int sensorPins[numCells] = { A6,A4,A3,A2,A1,A0 };
int bagPins[numCells][2][2] = {
	{{3,2}, {5,4}},		//1
	{{40,41}, {42,43}},	//2
	{{44,45}, {46,47}},	//3
	{{28,31}, {32,33}},	//4
	{{34,35}, {36,37}},	//5
	{{7,6}, {9,8}}		//6
};


// current path definition (adapt per application)

const int numPathAngles = 17;	//~~mPathStepSize
float pathAngles[numPathAngles][numCells] = {  //~~mPathAngleSteps
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


#endif