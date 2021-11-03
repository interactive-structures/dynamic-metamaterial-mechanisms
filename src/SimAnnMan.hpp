#ifndef SimAnnMan_hpp
#define SimAnnMan_hpp

#include "GridModel.h"
#include <float.h>
#include <map>
#include <tuple>

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

    // All cells to shear
    for (auto cell : bestModel.cells)
    {
      cell.type = SHEAR;
    }

    // Change some cells to rigid
    std::vector<int> toRigid;
    srand(time(NULL));
    while (toRigid.size() < sqrt(bestModel.cells.size()))
    {
      int candidate = rand() % bestModel.cells.size();
      bool accept = true;
      for (int i : toRigid)
      {
        if (i == candidate)
        {
          accept = false;
          break;
        }
      }
      if (accept) {toRigid.push_back(candidate);}
    }
    for (int i : toRigid)
    {
      bestModel.cells[i].type = RIGID;
    }

    workingModel = GridModel(bestModel);
    minError = DBL_MAX;
    workingError = DBL_MAX;
    outFolder = folder;
  };

  void runSimulatedAnnealing (int maxIterations = 100, double coolingFactor = 0.99);

private:
  std::tuple<double, double, double> calcObj(GridModel candidate, double pathNormSum);
};

#endif