#include "Animation.hpp"
#include "GridModel.h"
#include <fstream>
#include <float.h>

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
std::vector<std::vector<double>> get_angles(std::vector<GridResult> res, GridModel model) {
  std::vector<std::vector<double>> angles;
  for (auto frame : res) {
    std::vector<double> anglesThisFrame;
    for (auto cell : model.cells) {
      Eigen::Vector2d vec1 = frame.points[cell.vertices[1]] - frame.points[cell.vertices[0]];
      Eigen::Vector2d vec2 = frame.points[cell.vertices[3]] - frame.points[cell.vertices[0]];
      double angle = std::atan2(vec1[0]*vec2[1] - vec1[1]*vec2[0], vec1.dot(vec2));
      anglesThisFrame.push_back(angle);
    }
    angles.push_back(anglesThisFrame);
  }
  return angles;
}

int main(int argc, char *argv[])
{

  //std::cout << "loading" << std::endl;
  GridModel gm;
  gm.loadFromFile("../cells_3x3.txt");
  //std::cout << "loaded" << std::endl;

  gm.generateConstraintGraph();
  auto ret = optimize(gm, "../points/");

  auto cell_angles = get_angles(ret, gm);
  GridModel gm_copy(gm);

  // remove target paths
  gm_copy.targets = std::vector<int>();
  gm_copy.targetPaths = std::vector<std::vector<Eigen::Vector2d>>();

  // turn all shearing cells to active
  // for (GridCell &cell : gm_copy.cells) {
  //   if (cell.type == SHEAR) {
  //     cell.type = ACTIVE;
  //   }
  // }

  gm_copy.cells[0].type = ACTIVE;
  gm_copy.cells[1].type = ACTIVE;
  //gm_copy.cells[5].type = ACTIVE;
  //gm_copy.cells[6].type = ACTIVE;

  // std::cout << "Frame 0 angles: ";
  // for (double angle : cell_angles[0]) {
  //   std::cout << angle << " ";
  // }
  // std::cout << std::endl;

  auto angle_ret = optimizeActive(gm_copy, cell_angles, "../angle_points/");

  // SimuAn sa;
  // sa.simulatedAnnealing(gm);  

  // auto angles = get_angles(ret, gm);
  // std::cout << "Frame 0 angles: ";
  // for (double angle : angles[0]) {
  //   std::cout << angle << " ";
  // }
  // std::cout << std::endl;
  Animation original(gm, ret, 2, gm.targets);

  Animation animation(gm_copy, angle_ret, 2, gm.targets);
  // Animation animation(sa.best_model, sa.best_res);
  original.animate();
  animation.animate();
}
