#include <Servo.h>

#define SERVO_PIN_1 9
#define SERVO_PIN_2 8
#define SERVO_PIN_3 7

Servo s1;
Servo s2;
Servo s3;

void stopCode(){
  while(true) {
    continue;
  }
}

void moveServo(Servo servo1, int numDegPerMove, int degree, int del) {
  int deg = servo1.read();
  bool backwards = degree < deg; // true if we want to move backwards
  if (backwards) {
    while (deg > degree) {
      deg -= numDegPerMove;
      servo1.write(deg); 
      delay(del); 
    }
  }
  else {
    while (deg < degree) {
      deg += numDegPerMove;
      servo1.write(deg);
      delay(del);
    }
  }
}

void setup() {
  s1.attach(SERVO_PIN_1);
  s2.attach(SERVO_PIN_2);
  Serial.begin(9600);
}

void loop() {
  s1.write(90);
  s2.write(70);
  Serial.println(RAD_TO_DEG);
  float angles[] = {
    2.216, 1.90946,
    2.15872, 1.91481, 
    2.10112, 1.91894, 
    2.09318, 1.93277, 
    2.10927, 1.95192, 
    2.12426, 1.97115,
    2.15315, 1.98297, 
    2.21157, 1.97901,
    2.26991, 1.97374,
    2.32838, 1.96707,
    2.38719, 1.9589,
    2.4466, 1.94911,
    2.50689, 1.93755, 
    2.51597, 1.92031,
    2.49809, 1.90109, 
    2.47906, 1.88213,
    2.45892, 1.86341, 
    2.4377, 1.8449, 
    2.41544, 1.82659, 
    2.39216, 1.80847,
    2.33614, 1.8196,
    2.28034, 1.82925,
    2.22458, 1.8375,
    2.16869, 1.84442,
    2.11251, 1.85008,
    2.05588, 1.85453,
    1.99862, 1.85779,
    1.94057, 1.85989, 
    1.88154, 1.86085,
    1.82134, 1.86067, 
    1.84168, 1.87967,
    1.86086, 1.89865, 
    1.87889, 1.91761,
    1.89577, 1.93657,
    1.91152, 1.95553,
    1.92614, 1.97449,
    1.93962, 1.99345,
    1.95197, 2.01242,
    1.96319, 2.0314,
    1.97327, 2.05038,
    2.0338, 2.05128,
    2.09369, 2.05104,
    2.15315, 2.04961,
    2.21236, 2.04696,
    2.27154, 2.04301,
    2.33089, 2.03767,
    2.39065, 2.03085,
    2.45107, 2.02242,
    2.51247, 2.01219,
    2.57523, 1.99997
  };
//  delay(1000);
//  for (int i = 0; i < 50; i++) {
//    Serial.print(angles[2*i]);
//    Serial.println(angles[2*i+1]);
//    s1.write(RAD_TO_DEG * angles[2*i]);
//    s2.write(RAD_TO_DEG * angles[2*i+1]);
//    delay(1000);
//  }
  stopCode();
}
