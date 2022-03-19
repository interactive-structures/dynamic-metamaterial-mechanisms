#include "MultiAnimation.hpp"
#include <math.h>
#include <float.h>
#include <igl/opengl/glfw/Viewer.h>

#define PI 3.14159265

Eigen::MatrixXi MultiAnimation::get_edge_matrix()
{
  int total_edges = 0;
  for (auto gm : gms) {
    for (int i = 0; i < gm.cells.size(); i++)
    {
      total_edges += 4;
      if (gm.cells[i].type == RIGID)
      {
        total_edges += 2;
      }
      if (gm.cells[i].type == ACTIVE)
      {
        total_edges += 1;
      }
    }
  }
  
  Eigen::MatrixXi E = Eigen::MatrixXi::Zero(total_edges, 2);

  int index = 0;
  int offset = 0;
  for (auto gm : gms) {
    for (int i = 0; i < gm.cells.size(); i++)
    {
      // Add four boundary edges
      E(index, 0) = gm.cells[i].vertices(0) + offset;
      E(index, 1) = gm.cells[i].vertices(1) + offset;
      index++;
      E(index, 0) = gm.cells[i].vertices(1) + offset;
      E(index, 1) = gm.cells[i].vertices(2) + offset;
      index++;
      E(index, 0) = gm.cells[i].vertices(2) + offset;
      E(index, 1) = gm.cells[i].vertices(3) + offset;
      index++;
      E(index, 0) = gm.cells[i].vertices(3) + offset;
      E(index, 1) = gm.cells[i].vertices(0) + offset;
      index++;
      if (gm.cells[i].type == RIGID)
      { // Diagonal edges
        E(index, 0) = gm.cells[i].vertices(1) + offset;
        E(index, 1) = gm.cells[i].vertices(3) + offset;
        index++;
        E(index, 0) = gm.cells[i].vertices(0) + offset;
        E(index, 1) = gm.cells[i].vertices(2) + offset;
        index++;
      }
      if (gm.cells[i].type == ACTIVE)
      { // Single diagonal edges
        E(index, 0) = gm.cells[i].vertices(1) + offset;
        E(index, 1) = gm.cells[i].vertices(3) + offset;
        index++;
      }
    }
    offset += gm.points.size();
  }
  
  return E;
}

std::tuple<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXd, Eigen::MatrixXd> MultiAnimation::animate_path()
{
  frame = frame  % ress[0].size(); // properly bound frame
  int edges_set = 0;
  int points_set = 0;
  
  // Initialize Points and Edges
  int points_size = 0;
  int edges_size = grid_edges.rows();

  for (int i = 0; i < ress.size(); i++) {
    std::vector<GridResult> res = ress[i];
    std::vector<int> res_traces = to_trace[i];
    std::vector<std::vector<Eigen::Vector2d>> res_target_paths = target_paths[i];

    points_size += res[frame].points.size() + res_traces.size() * res.size();
    edges_size += res_traces.size() * (res.size() - 1);

    for (auto path : res_target_paths) {
      points_size += path.size();
      edges_size += path.size() - 1;
    }
  }
  
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(points_size, 3);
  Eigen::MatrixXi E = Eigen::MatrixXi::Zero(edges_size, 2);
  Eigen::MatrixXd tmp = Eigen::MatrixXd::Zero(1, 3);

  // Points on grid
  Eigen::MatrixXd bases = Eigen::MatrixXd::Zero(2, 3);
  bases(0,1) = DBL_MAX;
  Eigen::MatrixXd cog = Eigen::MatrixXd::Zero(1, 3);
  int total_points = 0;
  for (auto res : ress) {
    for (int i = 0; i < res[frame].points.size(); i++)
    {
      tmp(0, 0) = res[frame].points[i](0);
      tmp(0, 1) = res[frame].points[i](1);
      tmp(0, 2) = 0;
      P.row(points_set) = tmp * last_rotation; // Apply rotation
      cog = cog + P.row(points_set); // Sum to find center of gravity
      if (P(points_set, 1) < bases(0, 1)) { // find lowest point
        bases.row(0) = P.row(points_set);
      }

      points_set++;
    }
    total_points += res[frame].points.size();
  }
  
  cog = cog / total_points; // Average

  // find candidate rotations
  double rot_angle;
  if (cog(0, 0) > bases(0, 0)) { // falls to right, find min angle
    rot_angle = PI;
    for (int i = 0; i < points_set; i++) {
      if (bases.row(0) == P.row(i)) {continue;}
      double curr_rot = atan2(P(i, 1) - bases(0, 1), P(i, 0) - bases(0, 0));
      if (curr_rot < rot_angle) {
        rot_angle = curr_rot;
        bases.row(1) = P.row(i);
      }
    }
    rot_angle = -rot_angle;
  } else { // falls to left, find max angle
    rot_angle = 0;
    for (int i = 0; i < points_set; i++) {
      if (bases.row(0) == P.row(i)) {continue;}
      double curr_rot = atan2(P(i, 1) - bases(0, 1), P(i, 0) - bases(0, 0));
      if (curr_rot > rot_angle) {
        rot_angle = curr_rot;
        bases.row(1) = P.row(i);
      }
    }
    rot_angle = PI - rot_angle;
  }

  // Edges on grids
  E.topRows(grid_edges.rows()) = grid_edges;
  edges_set = grid_edges.rows();

  // Traced Paths
  for (int ind = 0; ind < ress.size(); ind++) { // by layer
    std::vector<GridResult> res = ress[ind];
    std::vector<int> res_traces = to_trace[ind];
    
    for (int i = 0; i < res_traces.size(); i++)
    { // ith traced path
      for (int j = 0; j < res.size(); j++)
      { // at time j
        tmp(0, 0) = res[j].points[res_traces[i]](0);
        tmp(0, 1) = res[j].points[res_traces[i]](1);
        tmp(0, 2) = 0;
        P.row(points_set) = tmp * last_rotation; // Apply rotation
        if (j > 0) {
          E(edges_set, 0) = points_set - 1;
          E(edges_set, 1) = points_set;
          edges_set++;
        }
        points_set++;
      }
    }
  }

  int target_edges_start = edges_set;

  // Target paths
  for (auto res_target_paths : target_paths) {
    for (auto path : res_target_paths)
    {
      bool first = true;
      for (auto point : path)
      {
        tmp(0, 0) = point[0];
        tmp(0, 1) = point[1];
        tmp(0, 2) = 0;
        P.row(points_set) = tmp * last_rotation; // Apply rotation
        if (!first) {
          E(edges_set, 0) = points_set - 1;
          E(edges_set, 1) = points_set;
          edges_set++;
        } else {
          first = false;
        }
        points_set++;
      }
    }
  }

  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity(3, 3); // construct rotation matrix
  rotation(0, 0) = cos(rot_angle);
  rotation(0, 1) = sin(rot_angle);
  rotation(1, 0) = -sin(rot_angle);
  rotation(1, 1) = cos(rot_angle);

  bases = bases * rotation; // rotate base

  assert(abs(bases(0,1) - bases(1,1)) < 0.001); // extract translation
  double translate_base = bases(0, 1);

  Eigen::MatrixXd translation = Eigen::MatrixXd::Zero(points_size, 3); // translation for all points
  translation.col(1).setOnes();
  translation = translation * (-translate_base);

  // Point Colors
  int point_color_size = 0;
  std::vector<int> anchor_inds = std::vector<int>();
  for (int i = 0; i < ress.size(); i++) {
    std::vector<GridResult> res = ress[i];
    GridModel gm = gms[i];

    for (int i : gm.anchors) {
      anchor_inds.push_back(point_color_size + i);
    }
    point_color_size += res[frame].points.size();
  }

  Eigen::MatrixXd PC = Eigen::MatrixXd::Zero(point_color_size, 3);
  for (int i : anchor_inds)
  {
    PC.row(i) = Eigen::RowVector3d(1, 0, 0);
  }

  // Edge Colors
  Eigen::MatrixXd EC = Eigen::MatrixXd::Zero(edges_size, 3);
  EC.bottomLeftCorner(edges_size - grid_edges.rows(), 1).setOnes();
  EC.bottomLeftCorner(edges_size - target_edges_start, 1).setZero();
  EC.bottomRightCorner(edges_size - target_edges_start, 1).setOnes();

  frame = (frame + 1) % ress[0].size(); // Increment frame

  last_rotation *= rotation; // update last rotation

  return std::make_tuple((P * rotation) + translation, E, PC, EC);
}

void MultiAnimation::animate()
{
  // Initialize viewer
  igl::opengl::glfw::Viewer viewer;
  viewer.data().point_size = 20;
  viewer.data().set_face_based(true);
  viewer.core().orthographic = true;
  viewer.core().toggle(viewer.data().show_lines);
  viewer.core().is_animating = true;
  viewer.core().background_color.setOnes();

  int curr_in_frame = 0;

  Eigen::RowVector3d base_color = Eigen::RowVector3d::Zero();
  base_color(0, 0) = 75;
  base_color(0, 2) = 130;

  Eigen::MatrixXd ground = Eigen::MatrixXd::Zero(2, 3);
  ground(0, 0) = -100;
  ground(1, 0) = 100;

  // Animation Callback
  viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer &) -> bool
  {
    curr_in_frame++;
    if (curr_in_frame == rate)
    { // slow controls frame rate
      // Get points and edges to draw
      auto tup = animate_path();
      auto points = std::get<0>(tup);
      auto edges = std::get<1>(tup);
      auto pointColors = std::get<2>(tup);
      auto edgeColors = std::get<3>(tup);
      // update points and edges
      viewer.data().set_points(points.topRows(pointColors.rows()), pointColors);
      viewer.data().set_edges(points, edges, edgeColors);
      // draw ground
      viewer.data().add_edges(ground.row(0), ground.row(1), base_color);
      curr_in_frame = 0;
    }

    return false;
  };

  viewer.launch();
}