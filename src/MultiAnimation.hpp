#ifndef MultiAnimation_hpp
#define MultiAnimation_hpp

#include "GridModel.h"
#include <tuple>
#include <utility>

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
  double last_translation;
  bool draw_ground;

  MultiAnimation(std::vector<GridModel> models, std::vector<std::vector<GridResult>> results, std::vector<std::vector<std::vector<Eigen::Vector2d>>> targets, int framerate=1, std::vector<std::vector<int>> traces=std::vector<std::vector<int>>(), bool ground = true)
  {
    frame = 0;
    rate = framerate;
    gms = models;
    ress = results;
    grid_edges = get_edge_matrix();
    std::pair<Eigen::MatrixXi, Eigen::MatrixXd> face_info = full_mesh_faces();
    F = face_info.first;
    C = face_info.second;
    to_trace = traces;
    target_paths = targets;
    last_rotation = Eigen::Matrix3d::Identity(3, 3);
    last_translation = 0;
    draw_ground = ground;
    set_targets();
  }
  std::vector<std::vector<double>> get_angles();
  void animate();
  void animate_mesh();
private:
  std::vector<GridModel> gms;
  std::vector<std::vector<GridResult>> ress;
  Eigen::MatrixXi grid_edges;
  Eigen::MatrixXi target_edges;
  Eigen::MatrixXd target_points;
  Eigen::MatrixXd target_colors;
  Eigen::MatrixXi F;
  Eigen::MatrixXd C;

  int ind_to_vtx(GridCell c, int ind);
  std::pair<Eigen::MatrixXi, Eigen::MatrixXd> cell_mesh_faces(GridCell c, int offset);
  std::pair<Eigen::MatrixXi, Eigen::MatrixXd> grid_mesh_faces(GridModel gm, int offset);
  Eigen::MatrixXi get_edge_matrix();
  std::pair<Eigen::MatrixXi, Eigen::MatrixXd> full_mesh_faces();
  Eigen::MatrixXd full_mesh_vertices();
  Eigen::MatrixXd full_mesh_vertices_no_floor();
  void set_targets();
  std::tuple<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXd, Eigen::MatrixXd> animate_path();
};

#endif