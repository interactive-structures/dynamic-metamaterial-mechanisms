#include "SimAnnMan.hpp"
#include <time.h>
#include <fstream>
#include <queue>
#include "Timer.hpp"
#include <filesystem>
#include <unordered_map>
#include <tuple>

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

GridModel generateRandomConfig(GridModel gm)
{
  int randTimes = 30;
  srand(time(NULL)); // Initialize rng
  GridModel cpy = GridModel(gm);
  for (int i = 0; i < randTimes; i++)
  {
    if (cpy.constraintGraph.size() > 1 && rand() % 2 == 0)
    {
      cpy.mergeComponents();
    }
    else
    {
      cpy.splitComponents();
    }
  }
  cpy.generateConstraintGraph();
  return cpy;
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
  std::filesystem::remove_all(folder);
  std::filesystem::create_directory(folder);

  // Get average step length for use in calculating objective
  std::vector<double> objPathNormSums;
  for (auto functionModel : bestModels)
  {
    double objPathNormSum = 0;
    for (int target = 0; target < functionModel.targetPaths.size(); target++)
    {
      double pathNorm = 0;
      for (int i = 1; i < functionModel.targetPaths[target].size(); i++)
      {
        double dx = functionModel.targetPaths[target][i][0] - functionModel.targetPaths[target][i - 1][0];
        double dy = functionModel.targetPaths[target][i][1] - functionModel.targetPaths[target][i - 1][1];
        pathNorm += sqrt(dx * dx + dy * dy);
      }
      objPathNormSum += pathNorm / (functionModel.targetPaths[target].size());
    }
    objPathNormSums.push_back(objPathNormSum);
  }

  

  // initialize restart parameters
  queue<double> err;
  int window = 10;
  double th = 0.01;
  int restart = 0;


  // initialize dictionary of objectives
  std::unordered_map<std::vector<bool>, std::tuple<double, double, double>> dict;


  // optimization loop
  for (int i = 0; i < maxIterations; i++)
  {
    // Create candidate model
    GridModel candidateBase = GridModel(workingModels[0]);
    candidateBase.generateConstraintGraph();
    if (candidateBase.constraintGraph.size() > 1 && rand() % 2 == 0)
    {
      candidateBase.mergeComponents();
    }
    else
    {
      candidateBase.splitComponents();
    }
    candidateBase.generateConstraintGraph();

    // Generate bitcode for candidate
    std::vector<bool> bitcode;
    for (auto c : candidateBase.cells)
    {
      if (c.type == RIGID) {bitcode.push_back(true);}
      else {bitcode.push_back(false);}
    }

    // push candidate modification to all models
    std::vector<GridModel> candidates;
    for (auto wm : workingModels)
    {
      GridModel candidate = GridModel(wm);
      candidate.cells = candidateBase.cells;
      candidate.generateConstraintGraph();
      candidates.push_back(candidate);
    }

    // Calculate simulated annealing values
    double newError;
    if (dict.find(bitcode) != dict.end()) // bitcode in dict
    {
      auto objTup = dict[bitcode];
      std::ofstream objOutFile;
      objOutFile.open(outFolder + "objectives.csv", std::ios_base::app);
      objOutFile << std::get<0>(objTup) << "," << std::get<1>(objTup) << "," << std::get<2>(objTup) << "\n";
      objOutFile.close();
      newError = std::get<2>(objTup);
    }
    else // bitcode not in dict
    {
      auto objTup = calcObj(candidates, objPathNormSums);
      dict[bitcode] = objTup;
      newError = std::get<2>(objTup);
    }
    
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
        //cout << "jump" << i << endl;
        GridModel active = candidateBase.addActiveCells();
        storeModelcpy(active, folder, i);
        restart++;

        // Remake random
        int num_cells = workingModels[0].cells.size();
        for (auto wm : workingModels) {
          for (auto cell : wm.cells)
          {
            cell.type = SHEAR;
          }
        }
        std::vector<int> toRigid;
        while (toRigid.size() < num_cells)
        {
          int candidate = rand() % num_cells;
          bool accept = true;
          for (int ii : toRigid)
          {
            if (ii == candidate)
            {
              accept = false;
              break;
            }
          }
          if (accept) {toRigid.push_back(candidate);}
        }
        for (auto wm : workingModels) {
          for (int i : toRigid)
          {
            wm.cells[i].type = RIGID;
          }
        }

        // Reset error queue
        err = std::queue<double>();
        continue;
      }
    }

    // Update accordingly
    if (newError < workingError)
    {
      workingModels = candidates;
      workingError = newError;
      // std::cout << "Update Working. ";
      if (newError < minError)
      {
        bestModels.clear();
        for (auto cand : candidates)
        {
          bestModels.push_back(GridModel(cand));
        }
        minError = newError;
        // std::cout << "New Best.";
      }
      // std::cout << std::endl;
    }
    else if ((double)rand() / RAND_MAX < forceAccept)
    {
      workingModels = candidates;
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

std::tuple<double, double, double> SimAnnMan::calcObj(std::vector<GridModel> candidates, std::vector<double> pathNormSums)
{
  double objective = 0;
  double totalPathObjective = 0;
  double dofObjective = 0;
  double totalAngleObjective = 0;

  for (int i = 0; i < candidates.size(); i++)
  {
    GridModel candidate = candidates[i];
    double pathNormSum = pathNormSums[i];

    double pathObjective = 0;
    double angleObjective = 0;
    auto ret = optimize(candidate, "");

    // from accuracy
    for (auto re : ret)
    {
      pathObjective += re.objError;
    }
    pathObjective = pathObjective / ret.size();
    pathObjective = pathObjective / pathNormSum;
    totalPathObjective += pathObjective;

    // from dof
    if (i == 0) {
      dofObjective = candidate.constraintGraph.size() / (2 * sqrt(candidate.cells.size()));
    }
    assert(dofObjective - candidate.constraintGraph.size() / (2 * sqrt(candidate.cells.size())) < 0.001);
    

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
  }

  double avgPathObjective = totalPathObjective / candidates.size();

  std::ofstream objOutFile;
  objOutFile.open(outFolder + "objectives.csv", std::ios_base::app);
  objective = pathWeight * avgPathObjective + dofWeight * dofObjective * dofObjective + 0.0 * totalAngleObjective;
  objOutFile << pathWeight * avgPathObjective << "," << dofWeight * dofObjective << "," << objective << "\n";
  objOutFile.close();
  return std::make_tuple(avgPathObjective, dofObjective, objective);
}