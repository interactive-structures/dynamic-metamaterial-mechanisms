# Build Instructions
Pull this repo

`git pull https://github.com/interactive-structures/dynamic-metamaterial-mechanisms`

Checkout this branch

`git checkout chipmunk-refactor-final`

Update submodules

`git submodule update --init`

Make and enter `build` directory

`mkdir build`

`cd build`

Run cmake

`cmake ../`

For linux/macos:

`make dynamic_mm_gui`

`./dynamic_mm_gui`

For Windows: open the project in Visual Studio and run it.

# Sensible Values for Simulation Parameters:
| name | meaning | value |
| -- | -- | -- |
| linkLength | length of simulated links | 0.4 |
