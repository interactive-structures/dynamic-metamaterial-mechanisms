#ifndef SimAnnMan_hpp
#define SimAnnMan_hpp

#include "GridModel.h"
#include <float.h>
#include <map>

#ifdef _WIN32
#define EIGEN_DONT_ALIGN_STATICALLY
#define EXPORT_DLL_COMMAND __declspec(dllexport)
#else
#define EXPORT_DLL_COMMAND
#endif

class EXPORT_DLL_COMMAND SimAnnMan
{

public:
  GridModel bestModel;
  GridModel workingModel;
  double minError;
  double workingError;
  std::string outFolder;

  SimAnnMan () {
    std::cout << "Error: No model specified" << std::endl;
  };

  SimAnnMan (GridModel gm, std::string folder = "") {
    bestModel = GridModel(gm);
    workingModel = GridModel(gm);
    minError = DBL_MAX;
    workingError = DBL_MAX;
    outFolder = folder;
  };

  void runSimulatedAnnealing (int maxIterations = 100, double coolingFactor = 0.99);

private:
  double calcObj(GridModel candidate);
};

#endif