int tolerance = 5;

int inflationAlgo(int s){
  int crossover;
  int offset;
  dic[s][currTarget] = angleLog[s][index];
  //  Serial.print("cell no "); Serial.print(s); Serial.print(" current target is "); Serial.println(dic[s][currTarget]);
  int cap0 = cap.filteredData(sensorPairs[s][0]);
  int cap1 = cap.filteredData(sensorPairs[s][1]);
  //  Serial.print(" cap 0 = "); Serial.print(cap0); Serial.print(" cap 1 = "); Serial.print(cap1);
  if (cap0 <= cap1){ //we should really look at which sensor gives a lower capacitance value at 90 degrees and orient the comparison that way. 
    dic[s][CAP] = cap0; 
    crossover = 0;
    offset = -180;
  } else {
    dic[s][CAP] = cap1;
    crossover = 1;
    offset = 0;
  }
  //  Serial.print("crossover = "); Serial.print(crossover); Serial.print(" offset = "); Serial.print(offset);
  dic[s][currAngle] = getAngleQuadratic(dic[s][CAP], crossover, s, offset);
  if (isnan(dic[s][currAngle])){dic[s][currAngle]=90;} //this was useful for one particular cell- see if it helps with others.
  Serial.print(" angle = "); Serial.println(dic[s][currAngle]);
  dic[s][currDiff] = dic[s][currTarget] - dic[s][currAngle];
  setValveStates(s); 
  dic[s][currDiff] = currTarget - currAngle;
  if (abs(dic[s][currDiff]) <= tolerance){
    return 1;
  } else {
    return 0;
  }
}

void setValveStates(int s){
  PIDs[s].Compute();
//  Serial.print("output: "); Serial.println(Output[s]);
  unsigned long now = millis();
  if ((now - windowStartTime[s]) >= abs(Output[s])){
    windowStartTime[s] = now;
    if (dic[s][currDiff] < 0){
      moveRight(s);
    } else{
      moveLeft(s);
    }
  }
  else{
//  Serial.print(" stop ");
  digitalWrite(dic[s][supplyValveA], LOW);
  digitalWrite(dic[s][exhaustValveA], LOW);
  digitalWrite(dic[s][supplyValveB], LOW);
  digitalWrite(dic[s][exhaustValveB], LOW);
  }
}

void moveRight(int s){
//  Serial.print(" right ");
  digitalWrite(dic[s][supplyValveA], HIGH);
  digitalWrite(dic[s][exhaustValveA], LOW);
  digitalWrite(dic[s][supplyValveB], LOW);
  digitalWrite(dic[s][exhaustValveB], HIGH);
}
void moveLeft(int s){
//  Serial.print(" left ");
  digitalWrite(dic[s][supplyValveA], LOW);
  digitalWrite(dic[s][exhaustValveA], HIGH);
  digitalWrite(dic[s][supplyValveB], HIGH);
  digitalWrite(dic[s][exhaustValveB], LOW);
  }
