#include "MMGrid.hpp"

class SimulatedAnnealing {
    public:
        SimulatedAnnealing(const string configfile, int numIterations, double startingTemp);
        MMGrid simulate();
}