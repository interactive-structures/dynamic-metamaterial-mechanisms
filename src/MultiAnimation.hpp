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
    F = full_mesh_faces();
    to_trace = traces;
    target_paths = targets;
    last_rotation = Eigen::Matrix3d::Identity(3, 3);
  }
  std::vector<std::vector<double>> get_angles();
  void animate();
  void animate_mesh();
private:
  std::vector<GridModel> gms;
  std::vector<std::vector<GridResult>> ress;
  Eigen::MatrixXi grid_edges;
  Eigen::MatrixXi F;

  int ind_to_vtx(GridCell c, int ind);
  Eigen::MatrixXi cell_mesh_faces(GridCell c, int offset);
  Eigen::MatrixXi grid_mesh_faces(GridModel gm, int offset);
  Eigen::MatrixXi get_edge_matrix();
  Eigen::MatrixXi full_mesh_faces();
  Eigen::MatrixXd full_mesh_vertices();
  std::tuple<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXd, Eigen::MatrixXd> animate_path();
};

#endif