#include "SimulatedAnnealingSet.hpp"
#include "UIModelData.hpp"

namespace SimulatedAnnealingNS {

    MMGrid mutate(MMGrid start) {
        ConstraintGraph cg(start.getRows(), start.getCols(), start.getCells());
        if (rand() % 2 == 0 && cg.dofs() > 1) {
            cg.mergeComponents();
        }
        else if (cg.dofs() == start.getRows() + start.getCols()) {
            cg.mergeComponents();
        }
        else {
            cg.splitComponents();
        }
        MMGrid result(start);
        result.setCells(start.getRows(), start.getRows(), cg.makeCells());
        return result;
    }
}


    SimulatedAnnealingSet::SimulatedAnnealingSet(std::vector<MMGrid> startGrids, double dofWeight, double pathWeight) : simGrids(startGrids), pathWeight(pathWeight), dofWeight(dofWeight) {}

    MMGrid SimulatedAnnealingSet::simulate(int numIterations, double coolingFactor) {
        srand(time(NULL));
        double startingTemp = numIterations / 3.0;
        double prevErr;
        double pathErr = 0;
        for (auto simGrid : simGrids) {
            pathErr += simGrid.getPathError();
        }
        ConstraintGraph cg(simGrids[0].getRows(), simGrids[0].getCols(), simGrids[0].getCells());
        double dofErr = cg.dofs();
        prevErr = pathErr * pathWeight + dofErr * dofWeight;
        for (int i = 0; i < numIterations; i++) {
            std::cout << "Iteration: " << i << endl;
            std::cout << "Previous weighted error is " << prevErr << std::endl;
            double acceptThresh = 1.0 / (1.0 + exp(prevErr / (startingTemp * pow(coolingFactor, i))));

            MMGrid candGrid = SimulatedAnnealingNS::mutate(simGrids[0]);
            pathErr = 0;
            int calcPathIndex = 0;
            for (auto simGrid : simGrids) {
                MMGrid tmp(simGrid);
                tmp.setCells(candGrid.getRows(), candGrid.getCols(), candGrid.getCells());
                pathErr += tmp.getPathError();
                UIModelData::allCalculatedPaths[calcPathIndex] = tmp.getCalculatedPaths();
                calcPathIndex++;
            }
            ConstraintGraph cg2(candGrid.getRows(), candGrid.getCols(), candGrid.getCells());
            dofErr = cg2.dofs();
            double newErr = pathErr * pathWeight + dofErr * dofWeight;
            std::cout << "New weighted error is " << newErr << std::endl;
            if (newErr < prevErr) {
                simGrids[0].setCells(candGrid.getRows(), candGrid.getCols(), candGrid.getCells());
                prevErr = newErr;
            }
            else if ((double)rand() / (double)RAND_MAX < acceptThresh) {
                simGrids[0].setCells(candGrid.getRows(), candGrid.getCols(), candGrid.getCells());
                prevErr = newErr;
            }
            else {
                simGrids[0].setCells(simGrids[0].getRows(), simGrids[0].getCols(), simGrids[0].getCells());
            }
        }
        
        cout << "all calc paths size" << endl;
        cout << UIModelData::allCalculatedPaths.size() << endl;
        return simGrids[0];
    }