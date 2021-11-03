# Robotic Metamaterials
 
## Dependencies
This project uses the following dependencies:
- [libigl](https://github.com/libigl/libigl/) for geometry processing. See the [libigl tutorial](https://libigl.github.io/tutorial/)
- [Ipopt](https://github.com/coin-or/Ipopt) for simulation with equality constraints
- [cppOpt](https://github.com/I3ck/cppOpt) for Simulated Annealing (currently added as dependency, but not executed in code)

Clone this repository with all dependencies, configured as submodules, using:
```
git clone --recursive <repo>
```

Check the [**OS specific build** instructions](##OS-specific-instructions) below before building the project.


## Build the project using CMake
As usual, create a build folder, navigate into that folder and call the build script from there. `CMakeLists.txt` in the root folder is the build script, which includes and links all dependencies. 

```
mkdir build
cd build
cmake ../
```

To compile on Unix-systems, additionally call
```
make
```


[](##OS-specific-instructions)
## OS specific instructions
### For MacOSX
First, this project does *not* work on M1 processor architecture.

MacOSX users need to **build Ipopt first**. Follow the instructions here: https://github.com/coin-or/Ipopt#getting-started

Tested on
- TBD

### For Windows
Ipopt is tricky to compile on Windows, which is why we deliver a binary with this repository. These precompiled libs are linked by CMake. 

*After* compiling, additionally set in VS project properties to use Intel MKL. If there are compiling problems, check that the Intel libraries are installed and added to the PATH environment variables.  

Tested on 
- Windows 10
- Visual Studio 2019 (MSVC 14)
- Configuration: RelWithDebInfo x64
- using Intel MKL ([Intel® oneAPI Base Toolkit](https://www.intel.com/content/www/us/en/developer/tools/oneapi/toolkits.html#base-kit))
- using Intel Fortran compiler ([Intel® oneAPI HPC Toolkit](https://www.intel.com/content/www/us/en/developer/tools/oneapi/toolkits.html#hpc-kit))

<!-- 
cmake ../ -DIPOPT_DIR:PATH=./dependencies/Ipopt/libWIN/lib


## 4. What should be linked
Include:

Linked:


Additional instructions: I use Intel MKL, instruct MSVC in VS project properties to use it (needs to be installed and added to Path, TODO: copy contents of path for reference!)
-->



---------------------------------

## Comments for developers

### Code: Simulated Annealing Process
1. Our error is divided into two parts: path accuracy error as a result of deviation from our desired path and a penalty on the degrees of freedom. We calculate our total error as a weighted sum of these two categories.

2. We save our error during each iteration of the simulated annealing process. If for the past 10 iterations, the variance of the weighted error is less than some threshold (a hyperparameter we can set to adjust the frequeny of the restart), we can restart by generating a random configuration.

3. During each restart, we will save our configuration before the restart.

4. After a fixed number of iterations, we will produce an output configuration with the least weighted error.

### Build: (Allegedly) How to point git submodule to a specific commit:

*1) add submodule:*
```
git submodule add <URL> <path>
```
example: ` git submodule add https://github.com/libigl/libigl.git ./dependencies/libigl `

*2) checkout the tag/commit we should track.* 
Note it is not encoded explicitly but apparently is tracked by the main project's git config. I haven't tested if this is true.

```
git checkout tags/<tagname> -b <tagname>-branch
```
example: ` git checkout tags/v2.3.0 -b v2.3.0-branch `

This switches to the tag commit and creates a new branch of that commit


### Untested: Consider these flags
```
CMAKE_CXX_FLAGS:STRING=-std=c++17 -stdlib=libc++ -I ../../ipopt/include
CMAKE_EXE_LINKER_FLAGS:STRING=-L../../ipopt/lib -lipopt
```


---------------------------------
## History

First version from 2018, as published at ACM CHI 2019.
- Publication: https://interactive-structures.org/publications/2019-05-understanding-mm/
- Code repository:  https://github.com/alexiiion/2018-MetamaterialMechanismsOptimization
    - Simulation in C++ using Ipopt
    - UI in C# using .NET WPF, which is Windows only
