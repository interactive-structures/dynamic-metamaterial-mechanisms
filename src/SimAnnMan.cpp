#include "SimAnnMan.hpp"
#include <time.h>

void SimAnnMan::runSimulatedAnnealing (int maxIterations, double coolingFactor)
{
  double startTemp = maxIterations / 3.0;
  srand(time(NULL)); // Initialize rng

  for (int i = 0; i < maxIterations; i++) {
    // Create candidate model
    GridModel candidate = GridModel(workingModel);
    candidate.generateConstraintGraph();
    if (candidate.constraintGraph.size() > 1 && rand() % 2 == 0) {
      candidate.mergeComponents();
    } else {
      candidate.splitComponents();
    }
    candidate.generateConstraintGraph();

    // Calculate simulated annealing values
    double newError = calcObj(candidate);
    double objective = 1.0 / calcObj(candidate);
    double forceAccept = 1.0 / (1.0 + exp(objective / (startTemp * pow(coolingFactor, i))));

    std::cout << "This Error: " << newError << ". ";

    // Update accordingly
    if (newError < workingError) {
      workingModel = candidate;
      workingError = newError;
      std::cout << "Update Working. ";
      if (newError < minError) {
        bestModel = GridModel(candidate);
        minError = newError;
        std::cout << "New Best.";
      }
      std::cout << std::endl;
    } else if ((double)rand() / RAND_MAX < forceAccept) {
      workingModel = candidate;
      workingError = newError;
      std::cout << "Force Accepted: " << forceAccept << std::endl;
    } else {
      std::cout << "Not Accepted: " << forceAccept << std::endl;
    }
  }
}

double SimAnnMan::calcObj(GridModel candidate)
{
    double error = 0;
    auto ret = optimize(candidate, "../points/");
    for (auto re : ret)
    {
        error += re.objError;
    }
    error += candidate.constraintGraph.size();

    return error;
}