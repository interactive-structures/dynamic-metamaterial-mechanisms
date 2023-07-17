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

MMGrid SimulatedAnnealing::simulate(int numIterations, double coolingFactor = .05) {
    srand(time(NULL));
    double startingTemp = numIterations / 3.0;
    double pathWeight = 1;
    double dofWeight = 5;
    for(int i = 0; i < numIterations; i++) {
        std::cout << "Iteration: " << i << endl;
        double pathErr = simGrid.getPathError();
        ConstraintGraph cg(simGrid.getRows(), simGrid.getCols(), simGrid.getCells());
        double dofErr = cg.dofs();
        double err = pathErr * pathWeight + dofErr * dofWeight;
        double acceptThresh = 1.0 / (1.0 + exp(err / (startingTemp * pow(coolingFactor, i))));

        MMGrid candGrid = mutate(simGrid);
        pathErr = candGrid.getPathError();
        ConstraintGraph cg2(candGrid.getRows(), candGrid.getCols(), candGrid.getCells());
        dofErr = cg2.dofs();
        double newErr = pathErr * pathWeight + dofErr * dofWeight;
        if(newErr < err) {
            simGrid = candGrid;
        }
        else if((double)rand() / (double)RAND_MAX < acceptThresh) {
            simGrid = candGrid;
        }
        simGrid.setCells(simGrid.getRows(), simGrid.getCols(), simGrid.getCells());
    }
    return simGrid;
}

