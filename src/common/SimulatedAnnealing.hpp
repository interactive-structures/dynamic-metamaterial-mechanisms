#include "MMGrid.hpp"

class SimulatedAnnealing {
    private:
        MMGrid simGrid;
        double pathWeight;
        double dofWeight;
    public:
        SimulatedAnnealing(string configfile);
        SimulatedAnnealing(MMGrid startGrid, double dofWeight, double pathWeight);
        MMGrid simulate(int numIterations, double coolingFactor = 0.05);
};