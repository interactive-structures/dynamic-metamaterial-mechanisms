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

    std::cout << "Iteration " << i << " Error: " << newError << ". ";

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
    double objective = 0;
    double pathObjective = 0;
    double dofObjective = 0;
    double angleObjective = 0;
    auto ret = optimize(candidate, "../points/");

    // from accuracy
    for (auto re : ret)
    {
        pathObjective += re.objError;
    }

    // from dof
    dofObjective += candidate.constraintGraph.size();

    // from angles
    std::vector<std::vector<double> > angles;
    for (auto frame : ret)
    {
      std::vector<double> anglesThisFrame;
      for (auto cell : candidate.cells)
      {
        Eigen::Vector2d vec1 = frame.points[cell.vertices[1]] - frame.points[cell.vertices[0]];
        Eigen::Vector2d vec2 = frame.points[cell.vertices[3]] - frame.points[cell.vertices[0]];
        double angle = std::atan2(vec1[0] * vec2[1] - vec1[1] * vec2[0], vec1.dot(vec2));
        anglesThisFrame.push_back(angle);
      }
      angles.push_back(anglesThisFrame);
    }
    for (int i = 1; i < angles.size(); i++)
    {
      for (int j = 0; j < angles[1].size(); j++)
      {
        angleObjective += std::abs(angles[i-1][j] - angles[i][j]);
      }
    }

    objective = 1.0 * pathObjective + 1.0 * dofObjective + 0.05 * angleObjective;
    return objective;
}