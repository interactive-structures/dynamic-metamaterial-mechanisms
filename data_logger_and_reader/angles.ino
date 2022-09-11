long previous = 0;
const int SAMPLE_RATE = 100;
int logIndex = 0;

double angleLog[NUM_CELLS][ARRAY_SIZE];

//float getAngleLinear(float input, int sensor_side, int s){
//  switch(sensor_side){
//    case 0:
//      return (input-dic[s][b_left])/dic[s][k_left];
//    case 1:
//      return (180 - (input-dic[s][b_right])/dic[s][k_right]);
//  }
//}

float getAngleQuadratic (float input, int sensor_side, int s, int offset){
  if (sensor_side == 0){
    float squareOfB = pow(dic[s][b_left], 2);
    float cWithInput = (dic[s][c_left] - input);  
    float fourAC = (4 * dic[s][a_left] * cWithInput);    
    float squareRootForQuadForm = sqrt(squareOfB - fourAC);  
    float answer = abs(offset + ((-dic[s][b_left] - squareRootForQuadForm)/(2*dic[s][a_left])));
    return answer;
  }
  else{
    float squareOfB = pow(dic[s][b_right], 2);  
    float cWithInput = (dic[s][c_right] - input);  
    float fourAC = (4 * dic[s][a_right] * cWithInput);    
    float squareRootForQuadForm = sqrt(squareOfB - fourAC);  
    float answer = abs(offset + ((-dic[s][b_right] - squareRootForQuadForm)/(2*dic[s][a_right])));
    return answer;
  }
}

void logAngles(){
  long now = millis();
  long elapsedTime = now - previous;
  int logCrossover;
  int logOffset;
  if (logIndex >= ARRAY_SIZE) {logIndex = 0;}
  if (elapsedTime >= SAMPLE_RATE){
    previous = now;
    for (int i = 0; i < NUM_CELLS; i++){
      int cap0 = cap.filteredData(sensorPairs[i][0]);
      int cap1 = cap.filteredData(sensorPairs[i][1]);
      if (cap0 <= cap1){ 
        dic[i][CAP] = cap0; 
        logCrossover = 0;
        logOffset = -180;
      } else {
        dic[i][CAP] = cap1;
        logCrossover = 1;
        logOffset = 0;
      }
      angleLog[i][logIndex] = getAngleQuadratic(dic[i][CAP], logCrossover, i, logOffset);
    }
    logIndex++;  
  }
}
