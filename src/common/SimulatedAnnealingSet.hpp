#include "MMGrid.hpp"

class SimulatedAnnealingSet {
    private:
        std::vector<MMGrid> simGrids;
        double pathWeight;
        double dofWeight;
    public:
        SimulatedAnnealingSet(std::vector<MMGrid> startGrids, double dofWeight, double pathWeight);
        MMGrid simulate(int numIterations, double coolingFactor = 0.05);
};