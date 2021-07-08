#ifndef SimuAn_hpp
#define SimuAn_hpp

#include "GridModel.h"
#include <float.h>
#include <map>

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
    std::map<double, GridModel> past_models;
    GridModel working_model;

    SimuAn()
    {
        std::cout << "Error: No starting model/path specified." << std::endl;
    };

    SimuAn(GridModel start)
    {
        working_model = GridModel(start);
        best_model = GridModel(start);
    };

    void simulatedAnnealing(double coolingFactor = 0.99, double startChance = 0.25);

private:
    std::vector<GridResult> res;
    int getRigidNum();
    double getError();
    double calcObj();
};

#endif