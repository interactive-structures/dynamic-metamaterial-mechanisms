# Build Instructions
Clone this branch of the repo!

`git clone --recursive -b chipmunk-refactor-final https://github.com/interactive-structures/dynamic-metamaterial-mechanisms`

In a terminal in your local copy of the repo,  make and enter `build` directory.

`mkdir build`

`cd build`

Run cmake

`cmake ../`

For linux/macos, make:

`make dynamic_mm_gui`

Then run:

`./dynamic_mm_gui`

For Windows: open the project in Visual Studio and run it.

# Sensible Values for Simulation Parameters:
| name | meaning | value |
| -- | -- | -- |
| linkLength | length of simulated links | 0.4 |
