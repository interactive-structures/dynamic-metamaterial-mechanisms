#include "SimuAn.hpp"
#include "cppOpt.h"

#include <fstream>
#include <stdlib.h>
#include <time.h>
#include <float.h>
#include <math.h>


int SimuAn::getRigidNum()
{
    int currRigid = 0;
    for (auto c : working_model.cells)
    {
        if (c.type == RIGID)
        {
            currRigid++;
        }
    }
    return currRigid;
}

double SimuAn::getError()
{
    double error = 0;
    auto ret = optimize(working_model, "../points/");
    for (auto re : ret)
    {
        error += re.objError;
    }

    return error;
}

double SimuAn::calcObj()
{
    double error = 0;
    auto ret = optimize(working_model, "../points/");
    for (auto re : ret)
    {
        error += re.objError;
    }
    error += working_model.constraintGraph.size();

    return error;
}

void SimuAn::simulatedAnnealing(double coolingFactor, double startChance)
{
    srand(time(NULL)); // Initialize rng
    least_error = FLT_MAX;

    auto toOptimize = [this](cppOpt::OptCalculation<double> &optCalculation)
    {
        double x = optCalculation.get_parameter("x");

        // find or generate grid associated with parameter
        if (past_models.find(x) != past_models.end()) {
            working_model = GridModel(past_models.find(x)->second);
            std::cout << "Previously seen model" << std::endl;
        } else {
            working_model.generateConstraintGraph();
            int dofs = working_model.constraintGraph.size();
            int target_dofs = 2 + floor(x * 4.0); //TODO: adjust

            std::cout << "Current DOFs: " << dofs << ". Target DOFs: " << target_dofs << ". x = " << x << std::endl;

            if (dofs < target_dofs) {
                while (working_model.constraintGraph.size() < target_dofs) {
                    working_model.splitComponents();
                    working_model.generateConstraintGraph();
                }
            } else if (dofs > target_dofs) {
                while (working_model.constraintGraph.size() > target_dofs) {
                    working_model.mergeComponents();
                    working_model.generateConstraintGraph();
                }
            } else {
                if (dofs > 1 && rand() % 2 == 0) {
                    working_model.mergeComponents();
                    working_model.splitComponents();
                    working_model.generateConstraintGraph();
                } else {
                    working_model.splitComponents();
                    working_model.mergeComponents();
                    working_model.generateConstraintGraph();
                }
            }
        }

        // add this grid to records
        past_models.insert(std::pair<double, GridModel>(x, GridModel(working_model)));

        // int cellSize = gm.cells.size();
        // int rigidNum = getRigidNum();

        // int targetRigidNum = int(x * cellSize + 0.5);
        // int iteration = 0;
        // int lastRigidNum, currRigidNum;

        // if (rigidNum == targetRigidNum)
        // {
        //     ;
        // }
        // else if (rigidNum < targetRigidNum)
        // {
        //     while (getRigidNum() < targetRigidNum)
        //     {
        //         std::cout << targetRigidNum << endl;
        //         std::cout << getRigidNum() << endl;

        //         lastRigidNum = getRigidNum();
        //         gm.mergeComponents(false);
        //         currRigidNum = getRigidNum();


        //         if (lastRigidNum == currRigidNum)
        //         {
        //             iteration++;
        //         }
        //         if (iteration > 5)
        //         {
        //             iteration = 0;
        //             gm.splitComponents();
        //             break;
        //         }
        //     }
        // }
        // else
        // {
        //     while (getRigidNum() > targetRigidNum)
        //     {
        //         std::cout << targetRigidNum << endl;
        //         std::cout << getRigidNum() << endl;
        //         lastRigidNum = getRigidNum();
        //         gm.splitComponents();
        //         currRigidNum = getRigidNum();
        //         if (lastRigidNum == currRigidNum)
        //         {
        //             iteration++;
        //         }
        //         if (iteration > 5)
        //         {
        //             iteration = 0;
        //             gm.mergeComponents(false);
        //             break;
        //         }
        //     }
        // }

        optCalculation.result = calcObj();
        std::cout << optCalculation.result;

        if (optCalculation.result < least_error)
        {
            best_model = GridModel(working_model);
            least_error = optCalculation.result;
            std::cout << ": New best model";
        }
        std::cout << std::endl;
    };

    cppOpt::OptBoundaries<double> optBoundaries;
    optBoundaries.add_boundary({0.0, 1.0, "x"});

    //number of calculations
    unsigned int maxCalculations = 100;

    //we want to find the minimum
    cppOpt::OptTarget optTarget = cppOpt::OptTarget::MINIMIZE;

    //how fast the simulated annealing algorithm slows down
    //http://en.wikipedia.org/wiki/Simulated_annealing
    //double coolingFactor = 0.99;

    //the chance in the beginning to follow bad solutions
    //double startChance = 0.25;

    //define your coordinator
    cppOpt::OptCoordinator<double, false> coordinator(
        maxCalculations,
        toOptimize,
        optTarget,
        0);

    //add simulated annealing as child
    coordinator.add_child(make_unique<cppOpt::OptSimulatedAnnealing<double> >(
        optBoundaries,
        coolingFactor,
        startChance));

    //let's go
    coordinator.run_optimisation();

    //print result
    cppOpt::OptCalculation<double> best = coordinator.get_best_calculation();
    cout << best.to_string_header() << endl;
    cout << best.to_string_values() << endl;
}