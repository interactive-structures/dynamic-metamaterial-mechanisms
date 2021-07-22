#include "SimAnnMan.hpp"
#include <time.h>
#include <fstream>
#include <queue>
#include "Timer.hpp"
#include <filesystem>

using namespace std;

void storeModelcpy(GridModel gm, std::string outFolder, int restart)
{
  std::ofstream gridOutFile;
  gridOutFile.open(outFolder + "output_model" + to_string(restart), std::ofstream::out | std::ofstream::trunc);

  gridOutFile << "#num_vertices #num_cells #num_anchors #index_inputvertex #num_inputpoints #index_outputvertex #num_outputpoints\n";
  gridOutFile << gm.points.size() << " " << gm.cells.size() << " " << gm.anchors.size() << " ";
  if (gm.targets.size() != 0)
  {
    gridOutFile << gm.targets[0] << " " << gm.targetPaths[0].size() << " ";
  }
  else
  {
    gridOutFile << "-1 0 ";
  }

  if (gm.inputs.size() != 0)
  {
    gridOutFile << gm.inputs[0] << " " << gm.inputPaths[0].size() << " \n";
  }
  else
  {
    gridOutFile << "-1 0 \n";
  }

  gridOutFile << "\n#vertices\n";
  for (auto p : gm.points)
  {
    gridOutFile << p[0] << " " << p[1] << "\n";
  }

  gridOutFile << "\n#anchors\n";
  for (auto a : gm.anchors)
  {
    gridOutFile << a << " ";
  }
  gridOutFile << "\n";

  gridOutFile << "\n#cells [type s=shear r=rigid a=actuating]\n";
  for (auto c : gm.cells)
  {
    if (c.type == RIGID)
    {
      gridOutFile << "r ";
    }
    else if (c.type == SHEAR)
    {
      gridOutFile << "s ";
    }
    else if (c.type == ACTIVE)
    {
      gridOutFile << "a ";
    }
    gridOutFile << c.vertices[0] << " " << c.vertices[1] << " " << c.vertices[2] << " " << c.vertices[3] << " \n";
  }

  if (gm.targets.size() != 0)
  {
    gridOutFile << "\n#input path\n";
    for (auto p : gm.targetPaths[0])
    {
      gridOutFile << p[0] << " " << p[1] << "\n";
    }
  }

  if (gm.inputs.size() != 0)
  {
    gridOutFile << "\n#output path\n";
    for (auto p : gm.inputPaths[0])
    {
      gridOutFile << p[0] << " " << p[1] << "\n";
    }
  }

  gridOutFile.close();
}

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
  srand(time(NULL));    // Initialize rng
  Timer timer("timer"); // Initialize timer
  if (!outFolder.empty())
  { // Intialize objective file
    std::ofstream objOutFile;
    objOutFile.open(outFolder + "objectives.csv", std::ofstream::out | std::ofstream::trunc);
    objOutFile << "Path Accuracy,DOFs,Error\n";
    objOutFile.close();
  }

  std::string folder = outFolder + "simAnn/";
  std::filesystem::create_directory(folder);

  double objPathNormSum = 0;
  for (int target = 0; target < workingModel.targetPaths.size(); target++)
  {
    double pathNorm = 0;
    for (int i = 1; i < workingModel.targetPaths[target].size(); i++)
    {
      double dx = workingModel.targetPaths[target][i][0] - workingModel.targetPaths[target][i - 1][0];
      double dy = workingModel.targetPaths[target][i][1] - workingModel.targetPaths[target][i - 1][1];
      pathNorm += sqrt(dx * dx + dy * dy);
    }
    objPathNormSum += pathNorm / (workingModel.targetPaths[target].size());
  }

  queue<double> err;
  int window = 10;
  double th = 0.01;
  int restart = 0;

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
    double newError = calcObj(candidate, objPathNormSum);
    double objective = 1.0 / newError;
    double forceAccept = 1.0 / (1.0 + exp(objective / (startTemp * pow(coolingFactor, i))));

    // std::cout << "Iteration " << i << " Error: " << newError << ". ";
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
        GridModel active = candidate.addActiveCells();
        storeModelcpy(active, folder, restart);
        restart++;
        continue;
      }
    }

    // Update accordingly
    if (newError < workingError)
    {
      workingModel = candidate;
      workingError = newError;
      // std::cout << "Update Working. ";
      if (newError < minError)
      {
        bestModel = GridModel(candidate);
        minError = newError;
        // std::cout << "New Best.";
      }
      // std::cout << std::endl;
    }
    else if ((double)rand() / RAND_MAX < forceAccept)
    {
      workingModel = candidate;
      workingError = newError;
      // std::cout << "Force Accepted: " << forceAccept << std::endl;
    }
    else
    {
      // std::cout << "Not Accepted: " << forceAccept << std::endl;
    }
  }

  int elapsed = timer.seconds();
  std::cout << "Simulated Annealing completed in " << elapsed << " seconds." << std::endl;
}

double SimAnnMan::calcObj(GridModel candidate, double pathNormSum)
{
  double objective = 0;
  double pathObjective = 0;
  double dofObjective = 0;
  double angleObjective = 0;
  auto ret = optimize(candidate, "");

  // from accuracy
  for (auto re : ret)
  {
    pathObjective += re.objError;
  }
  pathObjective = pathObjective / ret.size();
  pathObjective = pathObjective / pathNormSum;

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
  objOutFile.open(outFolder + "objectives.csv", std::ios_base::app);
  objective = pathObjective + dofObjective * dofObjective + 0.0 * angleObjective;
  objOutFile << pathObjective << "," << dofObjective << "," << objective << "\n";
  objOutFile.close();
  return objective;
}