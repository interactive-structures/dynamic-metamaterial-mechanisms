#include "SimulatedAnnealing.hpp"

MMGrid mutate(MMGrid start) {
    ConstraintGraph cg(start.getRows(), start.getCols(), start.getCells());
    if(rand() % 2 == 0 && cg.dofs() > 1) {
        cg.mergeComponents();
    }
    else if(cg.dofs() == start.getRows() + start.getCols()){
        cg.mergeComponents();
    }
    else {
        cg.splitComponents();
    }
    MMGrid result(start);
    result.setCells(start.getRows(), start.getRows(), cg.makeCells());
    return result;
}


SimulatedAnnealing::SimulatedAnnealing(string configfile) : simGrid(2, 2, std::vector<int>(4)){

    this->simGrid.loadFromFile(configfile);
}

SimulatedAnnealing::SimulatedAnnealing(MMGrid startGrid) : simGrid(startGrid) {}

MMGrid SimulatedAnnealing::simulate(int numIterations, double coolingFactor) {
    srand(time(NULL));
    double startingTemp = numIterations / 3.0;
    double pathWeight = 2;
    double dofWeight = 3;
    double prevErr;
    double pathErr = simGrid.getPathError();
    ConstraintGraph cg(simGrid.getRows(), simGrid.getCols(), simGrid.getCells());
    double dofErr = cg.dofs();
    prevErr = pathErr * pathWeight + dofErr * dofWeight;
    for(int i = 0; i < numIterations; i++) {
        std::cout << "Iteration: " << i << endl;
        std::cout << "Previous weighted error is " << prevErr << std::endl;
        double acceptThresh = 1.0 / (1.0 + exp(prevErr / (startingTemp * pow(coolingFactor, i))));

        MMGrid candGrid = mutate(simGrid);
        pathErr = candGrid.getPathError();
        ConstraintGraph cg2(candGrid.getRows(), candGrid.getCols(), candGrid.getCells());
        dofErr = cg2.dofs();
        double newErr = pathErr * pathWeight + dofErr * dofWeight;
        std::cout << "New weighted error is " << newErr << std::endl;
        if(newErr < prevErr) {
            simGrid.setCells(candGrid.getRows(), candGrid.getCols(), candGrid.getCells());
            prevErr = newErr;
        }
        else if((double)rand() / (double)RAND_MAX < acceptThresh) {
            simGrid.setCells(candGrid.getRows(), candGrid.getCols(), candGrid.getCells());
            prevErr = newErr;
        }
        else {
            simGrid.setCells(simGrid.getRows(), simGrid.getCols(), simGrid.getCells());
        }
        
    }
    return simGrid;
}

