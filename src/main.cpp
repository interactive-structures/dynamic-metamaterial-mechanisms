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

int main(int argc, char *argv[])
{

  //std::cout << "loading" << std::endl;
  GridModel gm;
  gm.loadFromFile("../cells.txt");
  //std::cout << "loaded" << std::endl;

  gm.generateConstraintGraph();
  // auto ret = optimize(gm, "../points/");

  SimuAn sa;
  sa.simulatedAnnealing(gm);  

  // auto angles = get_angles(ret, gm);
  // std::cout << "Frame 0 angles: ";
  // for (double angle : angles[0]) {
  //   std::cout << angle << " ";
  // }
  // std::cout << std::endl;

  Animation animation(sa.best_model, sa.best_res);
  animation.animate();
}
