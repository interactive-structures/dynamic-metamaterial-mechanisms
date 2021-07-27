#include "Animation.hpp"
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

void storeModel(GridModel gm, std::string outFolder) {
  std::ofstream gridOutFile;
  gridOutFile.open(outFolder + "output_model", std::ofstream::out | std::ofstream::trunc);

  gridOutFile << "#num_vertices #num_cells #num_anchors #index_inputvertex #num_inputpoints #index_outputvertex #num_outputpoints\n";
  gridOutFile << gm.points.size() << " " << gm.cells.size() << " " << gm.anchors.size() << " ";
  if (gm.targets.size() != 0) {
    gridOutFile << gm.targets[0] << " " << gm.targetPaths[0].size() << " ";
  } else {
    gridOutFile << "-1 0 ";
  }
  
  if (gm.inputs.size() != 0) {
    gridOutFile << gm.inputs[0] << " " << gm.inputPaths[0].size() << " \n";
  } else {
    gridOutFile << "-1 0 \n";
  }

  gridOutFile << "\n#vertices\n";
  for (auto p : gm.points) {
    gridOutFile << p[0] << " " << p[1] << "\n";
  }

  gridOutFile << "\n#anchors\n";
  for (auto a : gm.anchors) {
    gridOutFile << a << " ";
  }
  gridOutFile << "\n";

  gridOutFile << "\n#cells [type s=shear r=rigid a=actuating]\n";
  for (auto c : gm.cells) {
    if (c.type == RIGID) {
      gridOutFile << "r ";
    } else if (c.type == SHEAR) {
      gridOutFile << "s ";
    } else if (c.type == ACTIVE) {
      gridOutFile << "a ";
    }
    gridOutFile << c.vertices[0] << " " << c.vertices[1] << " " << c.vertices[2] << " " << c.vertices[3] << " \n";
  }

  if (gm.targets.size() != 0) {
    gridOutFile << "\n#input path\n";
    for (auto p : gm.targetPaths[0]) {
      gridOutFile << p[0] << " " << p[1] << "\n";
    }
  }

  if (gm.inputs.size() != 0) {
    gridOutFile << "\n#output path\n";
    for (auto p : gm.inputPaths[0]) {
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
    for (std::getline(frame, line); !line.empty(); std::getline(frame, line)) {
      anglesThisFrame.push_back(std::stod(line));
    }
    angles.push_back(anglesThisFrame);
    i++;
  }
  return angles;
}

int main(int argc, char *argv[])
{
  GridModel gm;
  gm.loadFromFile("../inputs/cells_7x7_big_line.txt");
  std::string folder = "../results/big_line/";
  std::string pointsFolder = folder + "points/";
  std::string anglesFolder = folder + "angles/";

  std::filesystem::create_directory(folder);
  std::filesystem::create_directory(pointsFolder);
  std::filesystem::create_directory(anglesFolder);

  // auto ret1 = optimize(gm, "");
  // Animation verify(gm, ret1, gm.targetPaths, 2, gm.targets);
  // verify.animate();
  // abort();


  SimAnnMan sa(gm, folder);
  sa.runSimulatedAnnealing(100, 0.97);

  gm = sa.bestModel;
  auto ret = optimize(gm, "");

  auto cell_angles = get_angles(ret, gm);
  GridModel gm_active = gm.addActiveCells();
  auto active_ret = optimizeActive(gm_active, cell_angles, pointsFolder, anglesFolder);
  storeModel(gm_active, folder);

  Animation animation(gm_active, active_ret, gm.targetPaths, 2, gm.targets);
  animation.animate();
  

  // Verify everything works by constructing gridmodel and angles from files
  GridModel gmFile;
  gmFile.loadFromFile(folder + "output_model");
  auto file_ret = optimizeActive(gmFile, anglesFromFolder(anglesFolder), "", "");

  // Write angles of active cells as csv to new file
  // std::ofstream activeAngleOutFile;
  // activeAngleOutFile.open(anglesFolder + "active.csv", std::ofstream::out | std::ofstream::trunc);
  // std::vector<int> activeCells;
  // std::string delim = "";
  // for (int i = 0; i < gmFile.cells.size(); i++) {
  //   if (gmFile.cells[i].type == ACTIVE) {
  //     activeCells.push_back(i);
  //     activeAngleOutFile << delim << "Cell " << i;
  //     delim = ",";
  //   }
  // }
  // activeAngleOutFile << "\n";
  // auto angles = anglesFromFolder(anglesFolder);
  // for (auto frame : angles) {
  //   delim = "";
  //   for (int cell : activeCells) {
  //     activeAngleOutFile << delim << frame[cell];
  //     delim = ",";
  //   }
  //   activeAngleOutFile << "\n";
  // }
  // activeAngleOutFile.close();


  Animation test(gmFile, file_ret, gm.targetPaths, 2, gm.targets);
  test.animate();
}
