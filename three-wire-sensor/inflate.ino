int inflate(int s, float input) { 
  double tolerance = 5;
  double currAngle = getAngle(dic[s][CAP],s);
  Serial.println(currAngle);
  double currTarget = angles[s][index];
  dic[s][currDiff] = currTarget - currAngle;
  bool inflateBag1 = true;
  PIDs[s].Compute();
  unsigned long now = millis();
  if (now - windowStartTime[s] > WindowSize)
  { //time to shift the Relay Window
    windowStartTime[s] += WindowSize;
  }

  //Output[s] is calculated in PID[s].Compute(); I don't know how that datum us calculated, but it is a number.
  if (Output[s] > now - windowStartTime[s]) {
    digitalWrite(dic[s][valveA], HIGH);
    digitalWrite(dic[s][valveB], LOW);
  } else {
    digitalWrite(dic[s][valveA], LOW);
    digitalWrite(dic[s][valveB], HIGH);
  }
  
  
//  Serial.print(", output: "); Serial.print(Output[s]);
//  Serial.print("Target Angle: "); Serial.print(currTarget);
//  Serial.print(", currAngle: "); Serial.print(currAngle);
//  Serial.print(",diff: "); Serial.println(dic[s][currDiff]);
//  
//  Serial.println();
  
  // move the index forward
  if(abs(dic[s][currDiff])<tolerance) {
    return 1;
  } else {
    return 0;
  }
}
