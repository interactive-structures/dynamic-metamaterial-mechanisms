#ifndef Animation_hpp
#define Animation_hpp

#include "GridModel.h"
#include <tuple>

#ifdef _WIN32
#define EIGEN_DONT_ALIGN_STATICALLY
#define EXPORT_DLL_COMMAND __declspec(dllexport)
#else
#define EXPORT_DLL_COMMAND
#endif

class EXPORT_DLL_COMMAND Animation
{
public:
  int frame;
  int rate;
  Animation(GridModel model, std::vector<GridResult> results, int framerate=1)
  {
    frame = 0;
    rate = framerate;
    gm = model;
    res = results;
    grid_edges = get_edge_matrix();
  }
  std::vector<std::vector<double>> get_angles();
  void animate();
private:
  GridModel gm;
  std::vector<GridResult> res;
  Eigen::MatrixXi grid_edges;

  Eigen::MatrixXi get_edge_matrix();
  std::tuple<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXd, Eigen::MatrixXd> animate_path();
};

#endif