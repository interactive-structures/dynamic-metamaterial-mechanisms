#include "Animation.hpp"
#include "GridModel.h"
#include "SimuAn.hpp"
#include <fstream>
#include <float.h>

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

int main(int argc, char *argv[])
{
  GridModel gm;
  gm.loadFromFile("../cells_3x3_quadrifolium.txt");

  //gm.generateConstraintGraph();
  //auto ret = optimize(gm, "../points/");

  SimuAn sa(gm);
  sa.simulatedAnnealing();

  gm = sa.best_model;
  auto ret = optimize(gm, "../points/");

  auto cell_angles = get_angles(ret, gm);
  GridModel gm_active = gm.addActiveCells();
  auto active_ret = optimizeActive(gm_active, cell_angles, "../angle_points/");

  // Code for 2x2 test case with each cell actuating at different times
  /**
  GridModel gm;
  gm.loadFromFile("../cells_2x2.txt");
  gm.generateConstraintGraph();
  GridModel gm_active = gm.addActiveCells();
  std::vector<std::vector<double>> cell_angles;
  for (int i = 0; i <= 25; i++)
  {
    std::vector<double> angles (4, PI/2.0);
    angles[0] = (PI/2.0) - (PI/4.0) * (i/25.0);
    cell_angles.push_back(angles);
  }
  for (int i = 0; i <= 25; i++)
  {
    std::vector<double> angles (4, PI/2.0);
    angles[0] = (PI/4.0);
    angles[2] = (PI/2.0) - (PI/4.0) * (i/25.0);
    cell_angles.push_back(angles);
  }
  for (int i = 0; i <= 25; i++)
  {
    std::vector<double> angles (4, PI/2.0);
    angles[0] = (PI/4.0);
    angles[2] = (PI/4.0);
    angles[3] = (PI/2.0) - (PI/4.0) * (i/25.0);
    cell_angles.push_back(angles);
  }
  auto active_ret = optimizeActive(gm, cell_angles, "../angle_points/");
  **/

  Animation original(gm, ret, 2, gm.targets);
  Animation animation(gm_active, active_ret, 2, gm.targets);
  // Animation animation(sa.best_model, sa.best_res);

  original.animate();
  animation.animate();
}
