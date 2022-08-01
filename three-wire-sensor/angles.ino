
float angles[NUM_CELLS][ARRAY_SIZE] = { //  cell0(k4),cell4(k4), 3x3
  {60, 90, 60, 120}
//  {60,75,90,105,120}
};

//takes the point-slope formula of from the spreadsheet and find the angle corresponding to a given capacitance

float getAngle(float input, int s){
  int sensor_side = crossover(s, sensor_pairs);
  switch(sensor_side){
    case 0:
      return (input-dic[s][b_left])/dic[s][k_left];
    case 1:
      return (180 - (input-dic[s][b_right])/dic[s][k_right]);
  }
}
