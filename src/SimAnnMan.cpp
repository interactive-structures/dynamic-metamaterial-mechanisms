#include "SimAnnMan.hpp"
#include <time.h>
#include <fstream>
#include <queue>

using namespace std;

double getVariane(queue<double> err)
{
  queue<double> cp1 = err;
  queue<double> cp2 = err;
  double sum = 0;
  double var = 0;
  while (!cp1.empty())
  {
    sum += cp1.front();
    cp1.pop();
  }
  double mean = sum / err.size();
  while (!cp2.empty())
  {
    double ele = cp2.front();
    cp2.pop();
    var += pow(ele - mean, 2);
  }
  return var / (err.size() - 1);
}

void SimAnnMan::runSimulatedAnnealing(int maxIterations, double coolingFactor)
{
  double startTemp = maxIterations / 3.0;
  srand(time(NULL)); // Initialize rng
  std::ofstream objOutFile;
  objOutFile.open("../objectives/objectives", std::ofstream::out | std::ofstream::trunc);
  objOutFile.close();
  queue<double> err;
  int window = 10;
  double th = 0.001;

  for (int i = 0; i < maxIterations; i++)
  {
    // Create candidate model
    GridModel candidate = GridModel(workingModel);
    candidate.generateConstraintGraph();
    if (candidate.constraintGraph.size() > 1 && rand() % 2 == 0)
    {
      candidate.mergeComponents();
    }
    else
    {
      candidate.splitComponents();
    }
    candidate.generateConstraintGraph();

    // Calculate simulated annealing values
    double newError = calcObj(candidate);
    double objective = 1.0 / newError;
    double forceAccept = 1.0 / (1.0 + exp(objective / (startTemp * pow(coolingFactor, i))));

    std::cout << "Iteration " << i << " Error: " << newError << ". ";
    if (err.size() < window)
    {
      err.push(newError);
    }
    else
    {
      err.push(newError);
      err.pop();
      if (getVariane(err) < th)
      {
        cout << "jump" << endl;
        continue;
      }
    }

    // Update accordingly
    if (newError < workingError)
    {
      workingModel = candidate;
      workingError = newError;
      std::cout << "Update Working. ";
      if (newError < minError)
      {
        bestModel = GridModel(candidate);
        minError = newError;
        std::cout << "New Best.";
      }
      std::cout << std::endl;
    }
    else if ((double)rand() / RAND_MAX < forceAccept)
    {
      workingModel = candidate;
      workingError = newError;
      std::cout << "Force Accepted: " << forceAccept << std::endl;
    }
    else
    {
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
  pathObjective = pathObjective / ret.size();

  // from dof
  dofObjective = candidate.constraintGraph.size() / (2 * sqrt(candidate.cells.size()));

  // from angles
  // std::vector<std::vector<double> > angles;
  // for (auto frame : ret)
  // {
  //   std::vector<double> anglesThisFrame;
  //   for (auto cell : candidate.cells)
  //   {
  //     Eigen::Vector2d vec1 = frame.points[cell.vertices[1]] - frame.points[cell.vertices[0]];
  //     Eigen::Vector2d vec2 = frame.points[cell.vertices[3]] - frame.points[cell.vertices[0]];
  //     double angle = std::atan2(vec1[0] * vec2[1] - vec1[1] * vec2[0], vec1.dot(vec2));
  //     anglesThisFrame.push_back(angle);
  //   }
  //   angles.push_back(anglesThisFrame);
  // }
  // for (int i = 1; i < angles.size(); i++)
  // {
  //   for (int j = 0; j < angles[1].size(); j++)
  //   {
  //     angleObjective += std::abs(angles[i-1][j] - angles[i][j]);
  //   }
  // }

  std::ofstream objOutFile;
  objOutFile.open("../objectives/objectives", std::ios_base::app);
  objOutFile << pathObjective << " " << dofObjective << "\n";
  objOutFile.close();
  objective = pathObjective + 0.2 * dofObjective * dofObjective + 0.0 * angleObjective;
  return objective;
}