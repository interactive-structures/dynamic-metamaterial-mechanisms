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
  std::vector<GridModel> bestModels;
  std::vector<GridModel> workingModels;
  double minError;
  double workingError;
  double pathWeight;
  double dofWeight;
  std::string outFolder;

  SimAnnMan () {
    std::cout << "Error: No model specified" << std::endl;
  };

  SimAnnMan (std::vector<GridModel> gms, std::string folder = "", double pw = 1, double dw = 1) {
    if (gms.size() == 0) {
      std::cout << "Error: No models in vector to SimAnnMan" << std::endl;
      exit(1);
    }

    pathWeight = pw;
    dofWeight = dw;

    int num_cells = gms[0].cells.size();

    bestModels.clear(); // clear best models
    for (auto gm : gms) // add starting config to list
    {
      bestModels.push_back(GridModel(gm));
    }

    // All cells to shear
    for (auto bgm : bestModels) {
      for (auto cell : bgm.cells)
      {
        cell.type = SHEAR;
      }
    }
    

    // Change some cells to rigid
    std::vector<int> toRigid;
    srand(time(NULL));
    while (toRigid.size() < sqrt(num_cells))
    {
      int candidate = rand() % num_cells;
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
    for (auto bgm : bestModels) {
      for (int i : toRigid)
      {
        bgm.cells[i].type = RIGID;
      }
    }


    workingModels.clear(); // clear best models
    for (auto bgm : bestModels) // add starting config to list
    {
      workingModels.push_back(GridModel(bgm));
    }

    minError = DBL_MAX;
    workingError = DBL_MAX;
    outFolder = folder;
  };

  void runSimulatedAnnealing (int maxIterations = 100, double coolingFactor = 0.99);

private:
  std::tuple<double, double, double> calcObj(std::vector<GridModel> candidates, std::vector<double> pathNormSums);
};

#endif