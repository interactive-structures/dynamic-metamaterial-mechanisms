#ifndef SimuAn_hpp
#define SimuAn_hpp

#include "GridModel.h"

#ifdef _WIN32
#define EIGEN_DONT_ALIGN_STATICALLY
#define EXPORT_DLL_COMMAND __declspec(dllexport)
#else
#define EXPORT_DLL_COMMAND
#endif

class EXPORT_DLL_COMMAND SimuAn
{

public:
    GridModel best_model;
    double least_error;
    //std::vector<GridResult> best_res;
    GridModel gm;

    void simulatedAnnealing(double coolingFactor = 0.99, double startChance = 0.25);

private:
    std::vector<GridResult> res;
    int getRigidNum();
    double getError();
};

#endif