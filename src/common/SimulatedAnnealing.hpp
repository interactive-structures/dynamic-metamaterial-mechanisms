#include "MMGrid.hpp"

class SimulatedAnnealing {
    private:
        MMGrid simGrid;
    public:
        SimulatedAnnealing(string configfile);
        SimulatedAnnealing(MMGrid startGrid);
        MMGrid simulate(int numIterations, double coolingFactor = 0.05);
};