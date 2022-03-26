#include "Animation.hpp"
#include "MultiAnimation.hpp"
#include "GridModel.h"
#include "SimuAn.hpp"
#include "SimAnnMan.hpp"
#include <fstream>
#include <float.h>
#include <fstream>
#include <filesystem>

#define PI 3.14159265

void printConstraintGraph(GridModel gm)
{
  auto cG = gm.constraintGraph;
  std::cout << std::endl
            << "Constraint Graph [" << std::endl;
  int compCount = 0;

  for (auto comp : cG)
  {
    std::cout << "Component " << compCount << std::endl;
    for (auto constraint : comp)
    {
      std::cout << "{";
      std::cout << "Cell: " << constraint.first.vertices[0] << ", " << constraint.first.vertices[1];
      std::cout << ", " << constraint.first.vertices[2] << ", " << constraint.first.vertices[3] << ". Constraints:";
      for (auto edge : constraint.second)
      {
        std::cout << " (" << edge.first << "," << edge.second << ")";
      }
      std::cout << "} ";
    }
    compCount++;
    std::cout << std::endl;
  }
  std::cout << "]" << std::endl;
}

// Returns vector of interior angles of each cell at each time step. By convention, the lower-left angle (vertices 3, 0, 1).
std::vector<std::vector<double> > get_angles(std::vector<GridResult> res, GridModel model)
{
  std::vector<std::vector<double> > angles;
  for (auto frame : res)
  {
    std::vector<double> anglesThisFrame;
    for (auto cell : model.cells)
    {
      Eigen::Vector2d vec1 = frame.points[cell.vertices[1]] - frame.points[cell.vertices[0]];
      Eigen::Vector2d vec2 = frame.points[cell.vertices[3]] - frame.points[cell.vertices[0]];
      double angle = std::atan2(vec1[0] * vec2[1] - vec1[1] * vec2[0], vec1.dot(vec2));
      anglesThisFrame.push_back(angle);
    }
    angles.push_back(anglesThisFrame);
  }
  return angles;
}

void storeModel(GridModel gm, std::string outFolder)
{
  std::ofstream gridOutFile;
  gridOutFile.open(outFolder + "output_model", std::ofstream::out | std::ofstream::trunc);

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

std::vector<std::vector<double> > anglesFromFolder(std::string anglesFolder)
{
  std::vector<std::vector<double> > angles;
  int i = 0;
  while (std::filesystem::exists(anglesFolder + "a" + std::to_string(i)))
  {
    std::ifstream frame(anglesFolder + "a" + std::to_string(i));
    std::vector<double> anglesThisFrame;
    std::string line;
    for (std::getline(frame, line); !line.empty(); std::getline(frame, line))
    {
      anglesThisFrame.push_back(std::stod(line));
    }
    angles.push_back(anglesThisFrame);
    i++;
  }
  return angles;
}

int main(int argc, char *argv[])
{ 
  GridModel gm1, gm2;
  gm1.loadFromFile("../example-data/inputs/cells_6x6_star_spike.txt"); // Specify input file
  // gm2.loadFromFile("../example-data/inputs/multi/cells_4x4_spike.txt");
  // cells_walk_offset_10x10.txt
  // cells_sync_walk_2_4x4.txt
  // cells_walk_offset_4x4.txt
  std::string folder = "../example-data/results/6x6_star_spike/";   // Specify output folder
  // walk_offset_10x10
  // sync_walk_2_4x4
  // walk_offset_4x4

  // Uncomment to view existing
  // GridModel anim_gm1, anim_gm2, anim_gm_file1, anim_gm_file2;

  // anim_gm1.loadFromFile("../example-data/inputs/layers/cells_layers_4x4_60.txt");
  // anim_gm2.loadFromFile("../example-data/inputs/layers/cells_layers_3x3_60.txt");

  // std::string animFolder1 = "../example-data/results/layers/layer_4x4/";
  // std::string anglesFolder1 = animFolder1 + "function_0/angles/";

  // std::string animFolder2 = "../example-data/results/layers/layer_3x3/";
  // std::string anglesFolder2 = animFolder2 + "function_0/angles/";

  // anim_gm_file1.loadFromFile(animFolder1 + "output_model");
  // anim_gm_file2.loadFromFile(animFolder2 + "output_model");

  // auto file_ret1 = optimizeActive(anim_gm_file1, anglesFromFolder(anglesFolder1), "", "");
  // auto file_ret2 = optimizeActive(anim_gm_file2, anglesFromFolder(anglesFolder2), "", "");

  // std::vector<GridModel> animation_gms;
  // std::vector<std::vector<GridResult>> animation_results;
  // std::vector<std::vector<std::vector<GridModel::Point>>> animation_target_paths;
  // std::vector<std::vector<int>> animation_targets;

  // animation_gms.push_back(anim_gm_file1);
  // animation_gms.push_back(anim_gm_file2);

  // animation_results.push_back(file_ret1);
  // animation_results.push_back(file_ret2);

  // animation_target_paths.push_back(anim_gm1.targetPaths);
  // animation_target_paths.push_back(anim_gm2.targetPaths);

  // animation_targets.push_back(anim_gm1.targets);
  // animation_targets.push_back(anim_gm2.targets);

  // MultiAnimation test(animation_gms, animation_results, animation_target_paths, 5, animation_targets);
  // test.animate_mesh();
  // abort();

  std::filesystem::create_directories(folder);

  // auto ret1 = optimize(gm, "");
  // Animation verify(gm, ret1, gm.targetPaths, 2, gm.targets);
  // verify.animate();
  // abort();

  std::vector<GridModel> gms; // place into vector
  gms.push_back(gm1);
  // gms.push_back(gm2); // pseudo multiple paths

  SimAnnMan sa(gms, folder, 1.0, 1.0);            // Initialize simulated annealing, specifying output folder
  sa.runSimulatedAnnealing(50, 0.97); // Run simulated annealing
  //sa.runSimulatedAnnealing(100, 0.97); // Run simulated annealing

  gms = sa.bestModels;           // Get best model from simulated annealing

  GridModel gm_active_base = GridModel(gms[0]).addActiveCells(); // add active cells

  for (int i = 0; i < gms.size(); i++) {
    GridModel gm = GridModel(gms[i]);

    std::string functionFolder = folder + "function_" + std::to_string(i) + "/";
    std::string pointsFolder = functionFolder + "points/";
    std::string anglesFolder = functionFolder + "angles/";

    std::filesystem::create_directory(functionFolder);
    std::filesystem::create_directory(pointsFolder);
    std::filesystem::create_directory(anglesFolder);


    auto ret = optimize(gm, ""); // run optimize to get grid position at each frame

    auto cell_angles = get_angles(ret, gm);                                               // get angles for each cell at each frame
    GridModel gm_active = GridModel(gm);
    gm_active.cells = gm_active_base.cells;                                               // add active cells
    auto active_ret = optimizeActive(gm_active, cell_angles, pointsFolder, anglesFolder); // Call optimizeActive to verify results with only control of actuating cells
    storeModel(gm_active, folder);                                                        // Store best model with actuating cells in results folder

    // set up animation: pass in as vector
    std::vector<GridModel> animation_gms;
    animation_gms.push_back(gm_active);

    std::vector<std::vector<GridResult>> animation_results;
    animation_results.push_back(active_ret);

    std::vector<std::vector<std::vector<GridModel::Point>>> animation_target_paths;
    animation_target_paths.push_back(gm.targetPaths);

    std::vector<std::vector<int>> animation_targets;
    animation_targets.push_back(gm.targets);

    // run animation
    MultiAnimation animation(animation_gms, animation_results, animation_target_paths, 2, animation_targets); 
    animation.animate_mesh();                                                       

    // Write angles of active cells as csv to new file
    std::ofstream activeAngleOutFile;
    activeAngleOutFile.open(anglesFolder + "active.csv", std::ofstream::out | std::ofstream::trunc);
    std::vector<int> activeCells;
    std::string delim = "";
    for (int i = 0; i < gm_active.cells.size(); i++)
    {
      if (gm_active.cells[i].type == ACTIVE)
      {
        activeCells.push_back(i);
        activeAngleOutFile << delim << "Cell " << i;
        delim = ",";
      }
    }
    activeAngleOutFile << "\n";
    auto angles = anglesFromFolder(anglesFolder);
    for (auto frame : angles)
    {
      delim = "";
      for (int cell : activeCells)
      {
        activeAngleOutFile << delim << frame[cell];
        delim = ",";
      }
      activeAngleOutFile << "\n";
    }
    activeAngleOutFile.close();
  }

  

  // Verify everything works by constructing gridmodel and angles from files
  // GridModel gmFile;
  // gmFile.loadFromFile(folder + "output_model");
  // auto file_ret = optimizeActive(gmFile, anglesFromFolder(anglesFolder), "", "");
  // Animation test(gmFile, file_ret, gm.targetPaths, 2, gm.targets);
  // test.animate();
}
