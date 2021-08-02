# Running Arduino code

## Electronics
- Items used: servos, power supply, jumper wires, Arduino board, breadboard
- In the example code, servo pins are defined to be 8 and 9-- change this to whatever corresponds to the physical setup
- If servos "stutter", it may be due to insufficient power-- hooking up the servos to a 5V power supply instead of Arduino's output might be better
- Depending on servo configuration, may need to subtract the desired angles from 180 instead of directly writing the angle to servo (dependant on servo direction)

## Running code
- Download Arduino IDE and place sketch in a folder of the same name
- Select appropriate board and USB output
- Verify and upload sketch
- To change the input file: Arduino unfortunately cannot read files from the computer, so the array in the sketch will have to be manually modified. array.py helps with this-- change the fileName at the top of the file and copy the console print to the sketch to replace the angles.

## 3D parts
Servo mounts:
https://cad.onshape.com/documents/38ad42bdb18868066bff46f1/w/60b1700f899d7c28ddf88365/e/e7e070e70bd1a219a74678d3
