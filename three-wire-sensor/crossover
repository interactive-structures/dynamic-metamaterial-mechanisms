//this function determines which sensor to read from each cell based off a pre-calibrated crossover point.

//to find crossover_point, find the intersection of the trendlines for both sensors in a cell. Use the Y value.
int crossover_point[NUM_CELLS] = {139};
int sensor_reading = 0;

int crossover(int index, int sensor_pairs[6][2]){
  int sensor_a = cap.filteredData(sensor_pairs[index][0]);;
  int sensor_b = cap.filteredData(sensor_pairs[index][1]);;
  if (sensor_a < crossover_point[index]){
    return 0;
  }
  else{
    return 1;
  }
}
