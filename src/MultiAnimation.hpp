#ifndef MultiAnimation_hpp
#define MultiAnimation_hpp

#include "GridModel.h"
#include <tuple>

#ifdef _WIN32
#define EIGEN_DONT_ALIGN_STATICALLY
#define EXPORT_DLL_COMMAND __declspec(dllexport)
#else
#define EXPORT_DLL_COMMAND
#endif

class EXPORT_DLL_COMMAND MultiAnimation
{
public:
  int frame;
  int rate;
  std::vector<std::vector<int>> to_trace;
  std::vector<std::vector<std::vector<Eigen::Vector2d>>> target_paths;
  Eigen::Matrix3d last_rotation;

  MultiAnimation(std::vector<GridModel> models, std::vector<std::vector<GridResult>> results, std::vector<std::vector<std::vector<Eigen::Vector2d>>> targets, int framerate=1, std::vector<std::vector<int>> traces=std::vector<std::vector<int>>())
  {
    frame = 0;
    rate = framerate;
    gms = models;
    ress = results;
    grid_edges = get_edge_matrix();
    to_trace = traces;
    target_paths = targets;
    last_rotation = Eigen::Matrix3d::Identity(3, 3);
  }
  std::vector<std::vector<double>> get_angles();
  void animate();
private:
  std::vector<GridModel> gms;
  std::vector<std::vector<GridResult>> ress;
  Eigen::MatrixXi grid_edges;

  Eigen::MatrixXi get_edge_matrix();
  std::tuple<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXd, Eigen::MatrixXd> animate_path();
};

#endif