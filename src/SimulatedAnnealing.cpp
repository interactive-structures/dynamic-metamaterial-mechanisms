#include "SimulatedAnnealing.hpp"

SimulatedAnnealing::SimulatedAnnealing(const string configfile, int numIterations, double startingTemp) {

}

MMGrid mutate(MMGrid start, double mutProb) {
    vector<int> cells = start.getCells();
    for(int i = 0; i < cells.size(); i++) {
        
    }
    return MMGrid(1,1,{1});
}
