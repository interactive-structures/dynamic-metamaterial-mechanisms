#include "MultiAnimation.hpp"
#include <math.h>
#include <float.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/resolve_duplicated_faces.h>
#include <utility>

#define PI 3.14159265

int MultiAnimation::ind_to_vtx(GridCell c, int ind) {
  return c.vertices(ind / 24) * 24 + ind % 24;
}

std::pair<Eigen::MatrixXi, Eigen::MatrixXd> MultiAnimation::cell_mesh_faces(GridCell c, int offset)
{
  Eigen::Matrix<int, Eigen::Dynamic, 3> edge_faces;
  Eigen::Matrix<double, Eigen::Dynamic, 3> edge_face_colors;
  edge_faces.resize(32, Eigen::NoChange);
  edge_face_colors.resize(32, Eigen::NoChange);
  edge_faces << ind_to_vtx(c, 1), ind_to_vtx(c, 24), ind_to_vtx(c, 27),
                ind_to_vtx(c, 1), ind_to_vtx(c, 27), ind_to_vtx(c, 2),
                ind_to_vtx(c, 2), ind_to_vtx(c, 27), ind_to_vtx(c, 31),
                ind_to_vtx(c, 2), ind_to_vtx(c, 31), ind_to_vtx(c, 6),
                ind_to_vtx(c, 5), ind_to_vtx(c, 6), ind_to_vtx(c, 31),
                ind_to_vtx(c, 5), ind_to_vtx(c, 31), ind_to_vtx(c, 28),
                ind_to_vtx(c, 1), ind_to_vtx(c, 5), ind_to_vtx(c, 28),
                ind_to_vtx(c, 1), ind_to_vtx(c, 28), ind_to_vtx(c, 24), // edge 1 end

                ind_to_vtx(c, 26), ind_to_vtx(c, 49), ind_to_vtx(c, 48),
                ind_to_vtx(c, 26), ind_to_vtx(c, 48), ind_to_vtx(c, 27),
                ind_to_vtx(c, 27), ind_to_vtx(c, 48), ind_to_vtx(c, 52),
                ind_to_vtx(c, 27), ind_to_vtx(c, 52), ind_to_vtx(c, 31),
                ind_to_vtx(c, 31), ind_to_vtx(c, 52), ind_to_vtx(c, 53),
                ind_to_vtx(c, 31), ind_to_vtx(c, 53), ind_to_vtx(c, 30),
                ind_to_vtx(c, 26), ind_to_vtx(c, 53), ind_to_vtx(c, 49),
                ind_to_vtx(c, 26), ind_to_vtx(c, 30), ind_to_vtx(c, 53), // edge 2 end

                ind_to_vtx(c, 73), ind_to_vtx(c, 48), ind_to_vtx(c, 51),
                ind_to_vtx(c, 73), ind_to_vtx(c, 51), ind_to_vtx(c, 74),
                ind_to_vtx(c, 74), ind_to_vtx(c, 51), ind_to_vtx(c, 55),
                ind_to_vtx(c, 74), ind_to_vtx(c, 55), ind_to_vtx(c, 78),
                ind_to_vtx(c, 78), ind_to_vtx(c, 55), ind_to_vtx(c, 52),
                ind_to_vtx(c, 78), ind_to_vtx(c, 52), ind_to_vtx(c, 77),
                ind_to_vtx(c, 73), ind_to_vtx(c, 77), ind_to_vtx(c, 52),
                ind_to_vtx(c, 73), ind_to_vtx(c, 52), ind_to_vtx(c, 48), // edge 3 end

                ind_to_vtx(c, 2), ind_to_vtx(c, 73), ind_to_vtx(c, 72),
                ind_to_vtx(c, 2), ind_to_vtx(c, 72), ind_to_vtx(c, 3),
                ind_to_vtx(c, 3), ind_to_vtx(c, 76), ind_to_vtx(c, 7),
                ind_to_vtx(c, 3), ind_to_vtx(c, 72), ind_to_vtx(c, 76),
                ind_to_vtx(c, 7), ind_to_vtx(c, 76), ind_to_vtx(c, 77),
                ind_to_vtx(c, 7), ind_to_vtx(c, 77), ind_to_vtx(c, 6),
                ind_to_vtx(c, 2), ind_to_vtx(c, 77), ind_to_vtx(c, 73),
                ind_to_vtx(c, 2), ind_to_vtx(c, 6), ind_to_vtx(c, 77); // edge 4 end
  edge_face_colors << 0.3, 0.3, 0.3,
                      0.3, 0.3, 0.3,
                      0.3, 0.3, 0.3,
                      0.3, 0.3, 0.3,
                      0.3, 0.3, 0.3,
                      0.3, 0.3, 0.3,
                      0.3, 0.3, 0.3,
                      0.3, 0.3, 0.3, // edge 1 end

                      0.3, 0.3, 0.3,
                      0.3, 0.3, 0.3,
                      0.3, 0.3, 0.3,
                      0.3, 0.3, 0.3,
                      0.3, 0.3, 0.3,
                      0.3, 0.3, 0.3,
                      0.3, 0.3, 0.3,
                      0.3, 0.3, 0.3, // edge 2 end

                      0.3, 0.3, 0.3,
                      0.3, 0.3, 0.3,
                      0.3, 0.3, 0.3,
                      0.3, 0.3, 0.3,
                      0.3, 0.3, 0.3,
                      0.3, 0.3, 0.3,
                      0.3, 0.3, 0.3,
                      0.3, 0.3, 0.3, // edge 3 end

                      0.3, 0.3, 0.3,
                      0.3, 0.3, 0.3,
                      0.3, 0.3, 0.3,
                      0.3, 0.3, 0.3,
                      0.3, 0.3, 0.3,
                      0.3, 0.3, 0.3,
                      0.3, 0.3, 0.3,
                      0.3, 0.3, 0.3; // edge 4 end
                
  if (c.type == RIGID) {
    edge_faces.conservativeResize(56, Eigen::NoChange);
    edge_face_colors.conservativeResize(56, Eigen::NoChange);
    edge_faces.bottomRows(24) << ind_to_vtx(c, 81), ind_to_vtx(c, 38), ind_to_vtx(c, 27),
                                 ind_to_vtx(c, 27), ind_to_vtx(c, 73), ind_to_vtx(c, 81),
                                 ind_to_vtx(c, 73), ind_to_vtx(c, 27), ind_to_vtx(c, 37),
                                 ind_to_vtx(c, 37), ind_to_vtx(c, 82), ind_to_vtx(c, 73), // top
                                 ind_to_vtx(c, 82), ind_to_vtx(c, 37), ind_to_vtx(c, 45),
                                 ind_to_vtx(c, 45), ind_to_vtx(c, 90), ind_to_vtx(c, 82), // upper side
                                 ind_to_vtx(c, 89), ind_to_vtx(c, 31), ind_to_vtx(c, 46),
                                 ind_to_vtx(c, 89), ind_to_vtx(c, 77), ind_to_vtx(c, 31), 
                                 ind_to_vtx(c, 77), ind_to_vtx(c, 45), ind_to_vtx(c, 31),
                                 ind_to_vtx(c, 77), ind_to_vtx(c, 90), ind_to_vtx(c, 45), // bottom
                                 ind_to_vtx(c, 38), ind_to_vtx(c, 81), ind_to_vtx(c, 89),
                                 ind_to_vtx(c, 38), ind_to_vtx(c, 89), ind_to_vtx(c, 46), // lower side
                                 // TL-BR end
                                 ind_to_vtx(c, 2), ind_to_vtx(c, 11), ind_to_vtx(c, 56),
                                 ind_to_vtx(c, 56), ind_to_vtx(c, 48), ind_to_vtx(c, 2),
                                 ind_to_vtx(c, 2), ind_to_vtx(c, 48), ind_to_vtx(c, 12),
                                 ind_to_vtx(c, 12), ind_to_vtx(c, 48), ind_to_vtx(c, 63), // top
                                 ind_to_vtx(c, 12), ind_to_vtx(c, 63), ind_to_vtx(c, 71),
                                 ind_to_vtx(c, 71), ind_to_vtx(c, 20), ind_to_vtx(c, 12), // upper side
                                 ind_to_vtx(c, 20), ind_to_vtx(c, 71), ind_to_vtx(c, 52),
                                 ind_to_vtx(c, 52), ind_to_vtx(c, 6), ind_to_vtx(c, 20), 
                                 ind_to_vtx(c, 6), ind_to_vtx(c, 52), ind_to_vtx(c, 64),
                                 ind_to_vtx(c, 64), ind_to_vtx(c, 19), ind_to_vtx(c, 6), // bottom
                                 ind_to_vtx(c, 11), ind_to_vtx(c, 19), ind_to_vtx(c, 64),
                                 ind_to_vtx(c, 64), ind_to_vtx(c, 56), ind_to_vtx(c, 11); // lower side
                                 // BL-TR end
    edge_face_colors.bottomRows(24) << 0.1, 0.1, 0.3,
                                       0.1, 0.1, 0.3,
                                       0.1, 0.1, 0.3,
                                       0.1, 0.1, 0.3, // top
                                       0.1, 0.1, 0.3,
                                       0.1, 0.1, 0.3, // upper side
                                       0.1, 0.1, 0.3,
                                       0.1, 0.1, 0.3, 
                                       0.1, 0.1, 0.3,
                                       0.1, 0.1, 0.3, // bottom
                                       0.1, 0.1, 0.3,
                                       0.1, 0.1, 0.3, // lower side
                                       // TL-BR end
                                       0.1, 0.1, 0.3,
                                       0.1, 0.1, 0.3,
                                       0.1, 0.1, 0.3,
                                       0.1, 0.1, 0.3, // top
                                       0.1, 0.1, 0.3,
                                       0.1, 0.1, 0.3, // upper side
                                       0.1, 0.1, 0.3,
                                       0.1, 0.1, 0.3, 
                                       0.1, 0.1, 0.3,
                                       0.1, 0.1, 0.3, // bottom
                                       0.1, 0.1, 0.3,
                                       0.1, 0.1, 0.3; // lower side
                                       // BL-TR end
  } else if (c.type == ACTIVE) {
    edge_faces.conservativeResize(44, Eigen::NoChange);
    edge_face_colors.conservativeResize(44, Eigen::NoChange);
    edge_faces.bottomRows(12) << ind_to_vtx(c, 81), ind_to_vtx(c, 38), ind_to_vtx(c, 27),
                                 ind_to_vtx(c, 27), ind_to_vtx(c, 73), ind_to_vtx(c, 81),
                                 ind_to_vtx(c, 73), ind_to_vtx(c, 27), ind_to_vtx(c, 37),
                                 ind_to_vtx(c, 37), ind_to_vtx(c, 82), ind_to_vtx(c, 73), // top
                                 ind_to_vtx(c, 82), ind_to_vtx(c, 37), ind_to_vtx(c, 45),
                                 ind_to_vtx(c, 45), ind_to_vtx(c, 90), ind_to_vtx(c, 82), // upper side
                                 ind_to_vtx(c, 89), ind_to_vtx(c, 31), ind_to_vtx(c, 46),
                                 ind_to_vtx(c, 89), ind_to_vtx(c, 77), ind_to_vtx(c, 31), 
                                 ind_to_vtx(c, 77), ind_to_vtx(c, 45), ind_to_vtx(c, 31),
                                 ind_to_vtx(c, 77), ind_to_vtx(c, 90), ind_to_vtx(c, 45), // bottom
                                 ind_to_vtx(c, 38), ind_to_vtx(c, 81), ind_to_vtx(c, 89),
                                 ind_to_vtx(c, 38), ind_to_vtx(c, 89), ind_to_vtx(c, 46); // lower side
    edge_face_colors.bottomRows(12) << 0.3, 0.0, 0.0,
                                       0.3, 0.0, 0.0,
                                       0.3, 0.0, 0.0,
                                       0.3, 0.0, 0.0, // top
                                       0.3, 0.0, 0.0,
                                       0.3, 0.0, 0.0, // upper side
                                       0.3, 0.0, 0.0,
                                       0.3, 0.0, 0.0, 
                                       0.3, 0.0, 0.0,
                                       0.3, 0.0, 0.0, // bottom
                                       0.3, 0.0, 0.0,
                                       0.3, 0.0, 0.0; // lower side
  }
  
  edge_faces = (edge_faces.array() + offset).matrix();

  return std::make_pair(edge_faces, edge_face_colors);
}

std::pair<Eigen::MatrixXi, Eigen::MatrixXd> MultiAnimation::grid_mesh_faces(GridModel gm, int offset) {
  Eigen::Matrix<int, Eigen::Dynamic, 3> grid_faces;
  Eigen::Matrix<double, Eigen::Dynamic, 3> grid_face_colors;
  for (auto c : gm.cells) {
    std::pair<Eigen::MatrixXi, Eigen::MatrixXd> cell_info = cell_mesh_faces(c, offset);
    Eigen::MatrixXi cell_faces = cell_info.first;
    Eigen::MatrixXd cell_face_colors = cell_info.second;
    grid_faces.conservativeResize(grid_faces.rows() + cell_faces.rows(), Eigen::NoChange);
    grid_face_colors.conservativeResize(grid_face_colors.rows() + cell_face_colors.rows(), Eigen::NoChange);
    grid_faces.bottomRows(cell_faces.rows()) = cell_faces;
    grid_face_colors.bottomRows(cell_face_colors.rows()) = cell_face_colors;
  }
  return std::make_pair(grid_faces, grid_face_colors);
}

std::pair<Eigen::MatrixXi, Eigen::MatrixXd> MultiAnimation::full_mesh_faces() {
  int offset = 0;
  Eigen::Matrix<int, Eigen::Dynamic, 3> F;
  Eigen::Matrix<double, Eigen::Dynamic, 3> C;
  for (auto gm : gms) {
    std::pair<Eigen::MatrixXi, Eigen::MatrixXd> grid_info = grid_mesh_faces(gm, offset);
    Eigen::MatrixXi grid_faces = grid_info.first;
    Eigen::MatrixXd grid_face_colors = grid_info.second;
    F.conservativeResize(F.rows() + grid_faces.rows(), Eigen::NoChange);
    C.conservativeResize(C.rows() + grid_face_colors.rows(), Eigen::NoChange);
    F.bottomRows(grid_faces.rows()) = grid_faces;
    C.bottomRows(grid_face_colors.rows()) = grid_face_colors;
    offset += 24 * gm.points.size();
  }

  // Floor mesh
  F.conservativeResize(F.rows() + 12, Eigen::NoChange);
  C.conservativeResize(C.rows() + 12, Eigen::NoChange);
  F.bottomRows(12) << offset, offset + 1, offset + 2,
                      offset, offset + 2, offset + 3, // top
                      offset + 4, offset + 1, offset, 
                      offset + 4, offset + 5, offset + 1, // front
                      offset + 3, offset + 2, offset + 6, 
                      offset + 3, offset + 6, offset + 7, // back
                      offset, offset + 3, offset + 7, 
                      offset, offset + 7, offset + 4, // left
                      offset + 2, offset + 1, offset + 5, 
                      offset + 2, offset + 5, offset + 6, // right
                      offset + 4, offset + 7, offset + 5,
                      offset + 5, offset + 7, offset + 6; // bottom
  C.bottomRows(12) << 0, 0, 0,
                      0, 0, 0, // top
                      0, 0, 0,
                      0, 0, 0, // front
                      0, 0, 0,
                      0, 0, 0, // back
                      0, 0, 0,
                      0, 0, 0, // left
                      0, 0, 0,
                      0, 0, 0, // right
                      0, 0, 0,
                      0, 0, 0; // bottom

  return std::make_pair(F, C);
}

Eigen::MatrixXd MultiAnimation::full_mesh_vertices() {
  frame = frame % ress[0].size(); // properly bound current frame

  int v_per_p = 24;
  int num_vertices = 0;
  for (auto gm : gms) {
    num_vertices += gm.points.size();
  }

  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(num_vertices, 3);

  num_vertices *= v_per_p;

  Eigen::MatrixXd V = Eigen::MatrixXd::Zero(num_vertices, 3);

  Eigen::MatrixXd tmp = Eigen::MatrixXd::Zero(1, 3);
  Eigen::MatrixXd bases = Eigen::MatrixXd::Zero(2, 3);
  bases(0,1) = DBL_MAX;
  Eigen::MatrixXd cog = Eigen::MatrixXd::Zero(1, 3);

  int layer = 0;
  int offset = 0;
  for (auto res : ress) {
    for (int i = 0; i < res[frame].points.size(); i++)
    {
      // cog and rotation calculations
      tmp(0, 0) = res[frame].points[i](0);
      tmp(0, 1) = res[frame].points[i](1);
      tmp(0, 2) = 0;
      tmp = tmp * last_rotation;
      P.row(offset + i) = tmp;
      cog = cog + tmp; // Sum to find center of gravity
      if (tmp(0, 1) < bases(0, 1)) { // track lowest point
        bases.row(0) = tmp;
      }

      // actually create the points

      // 4 upper layer points
      tmp(0, 0) = res[frame].points[i](0) - 0.01;
      tmp(0, 1) = res[frame].points[i](1) - 0.01;
      tmp(0, 2) = layer + 0.25;
      tmp = tmp * last_rotation;
      V.row(v_per_p * offset + v_per_p * i) = tmp;

      tmp(0, 0) = res[frame].points[i](0) + 0.01;
      tmp(0, 1) = res[frame].points[i](1) - 0.01;
      tmp(0, 2) = layer + 0.25;
      tmp = tmp * last_rotation;
      V.row(v_per_p * offset + v_per_p * i + 1) = tmp;

      tmp(0, 0) = res[frame].points[i](0) + 0.01;
      tmp(0, 1) = res[frame].points[i](1) + 0.01;
      tmp(0, 2) = layer + 0.25;
      tmp = tmp * last_rotation;
      V.row(v_per_p * offset + v_per_p * i + 2) = tmp;

      tmp(0, 0) = res[frame].points[i](0) - 0.01;
      tmp(0, 1) = res[frame].points[i](1) + 0.01;
      tmp(0, 2) = layer + 0.25;
      tmp = tmp * last_rotation;
      V.row(v_per_p * offset + v_per_p * i + 3) = tmp;

      // 4 lower layer points
      tmp(0, 0) = res[frame].points[i](0) - 0.01;
      tmp(0, 1) = res[frame].points[i](1) - 0.01;
      tmp(0, 2) = layer - 0.25;
      tmp = tmp * last_rotation;
      V.row(v_per_p * offset + v_per_p * i + 4) = tmp;

      tmp(0, 0) = res[frame].points[i](0) + 0.01;
      tmp(0, 1) = res[frame].points[i](1) - 0.01;
      tmp(0, 2) = layer - 0.25;
      tmp = tmp * last_rotation;
      V.row(v_per_p * offset + v_per_p * i + 5) = tmp;

      tmp(0, 0) = res[frame].points[i](0) + 0.01;
      tmp(0, 1) = res[frame].points[i](1) + 0.01;
      tmp(0, 2) = layer - 0.25;
      tmp = tmp * last_rotation;
      V.row(v_per_p * offset + v_per_p * i + 6) = tmp;

      tmp(0, 0) = res[frame].points[i](0) - 0.01;
      tmp(0, 1) = res[frame].points[i](1) + 0.01;
      tmp(0, 2) = layer - 0.25;
      tmp = tmp * last_rotation;
      V.row(v_per_p * offset + v_per_p * i + 7) = tmp;

      // 8 upper layer crosses
      tmp(0, 0) = res[frame].points[i](0) - 0.01;
      tmp(0, 1) = res[frame].points[i](1) - 0.02;
      tmp(0, 2) = layer + 0.25;
      tmp = tmp * last_rotation;
      V.row(v_per_p * offset + v_per_p * i + 8) = tmp;

      tmp(0, 0) = res[frame].points[i](0) + 0.01;
      tmp(0, 1) = res[frame].points[i](1) - 0.02;
      tmp(0, 2) = layer + 0.25;
      tmp = tmp * last_rotation;
      V.row(v_per_p * offset + v_per_p * i + 9) = tmp;

      tmp(0, 0) = res[frame].points[i](0) + 0.02;
      tmp(0, 1) = res[frame].points[i](1) - 0.01;
      tmp(0, 2) = layer + 0.25;
      tmp = tmp * last_rotation;
      V.row(v_per_p * offset + v_per_p * i + 10) = tmp;

      tmp(0, 0) = res[frame].points[i](0) + 0.02;
      tmp(0, 1) = res[frame].points[i](1) + 0.01;
      tmp(0, 2) = layer + 0.25;
      tmp = tmp * last_rotation;
      V.row(v_per_p * offset + v_per_p * i + 11) = tmp;

      tmp(0, 0) = res[frame].points[i](0) + 0.01;
      tmp(0, 1) = res[frame].points[i](1) + 0.02;
      tmp(0, 2) = layer + 0.25;
      tmp = tmp * last_rotation;
      V.row(v_per_p * offset + v_per_p * i + 12) = tmp;

      tmp(0, 0) = res[frame].points[i](0) - 0.01;
      tmp(0, 1) = res[frame].points[i](1) + 0.02;
      tmp(0, 2) = layer + 0.25;
      tmp = tmp * last_rotation;
      V.row(v_per_p * offset + v_per_p * i + 13) = tmp;

      tmp(0, 0) = res[frame].points[i](0) - 0.02;
      tmp(0, 1) = res[frame].points[i](1) + 0.01;
      tmp(0, 2) = layer + 0.25;
      tmp = tmp * last_rotation;
      V.row(v_per_p * offset + v_per_p * i + 14) = tmp;

      tmp(0, 0) = res[frame].points[i](0) - 0.02;
      tmp(0, 1) = res[frame].points[i](1) - 0.01;
      tmp(0, 2) = layer + 0.25;
      tmp = tmp * last_rotation;
      V.row(v_per_p * offset + v_per_p * i + 15) = tmp;

      // 8 lower layer crosses
      tmp(0, 0) = res[frame].points[i](0) - 0.01;
      tmp(0, 1) = res[frame].points[i](1) - 0.02;
      tmp(0, 2) = layer - 0.25;
      tmp = tmp * last_rotation;
      V.row(v_per_p * offset + v_per_p * i + 16) = tmp;

      tmp(0, 0) = res[frame].points[i](0) + 0.01;
      tmp(0, 1) = res[frame].points[i](1) - 0.02;
      tmp(0, 2) = layer - 0.25;
      tmp = tmp * last_rotation;
      V.row(v_per_p * offset + v_per_p * i + 17) = tmp;

      tmp(0, 0) = res[frame].points[i](0) + 0.02;
      tmp(0, 1) = res[frame].points[i](1) - 0.01;
      tmp(0, 2) = layer - 0.25;
      tmp = tmp * last_rotation;
      V.row(v_per_p * offset + v_per_p * i + 18) = tmp;

      tmp(0, 0) = res[frame].points[i](0) + 0.02;
      tmp(0, 1) = res[frame].points[i](1) + 0.01;
      tmp(0, 2) = layer - 0.25;
      tmp = tmp * last_rotation;
      V.row(v_per_p * offset + v_per_p * i + 19) = tmp;

      tmp(0, 0) = res[frame].points[i](0) + 0.01;
      tmp(0, 1) = res[frame].points[i](1) + 0.02;
      tmp(0, 2) = layer - 0.25;
      tmp = tmp * last_rotation;
      V.row(v_per_p * offset + v_per_p * i + 20) = tmp;

      tmp(0, 0) = res[frame].points[i](0) - 0.01;
      tmp(0, 1) = res[frame].points[i](1) + 0.02;
      tmp(0, 2) = layer - 0.25;
      tmp = tmp * last_rotation;
      V.row(v_per_p * offset + v_per_p * i + 21) = tmp;

      tmp(0, 0) = res[frame].points[i](0) - 0.02;
      tmp(0, 1) = res[frame].points[i](1) + 0.01;
      tmp(0, 2) = layer - 0.25;
      tmp = tmp * last_rotation;
      V.row(v_per_p * offset + v_per_p * i + 22) = tmp;

      tmp(0, 0) = res[frame].points[i](0) - 0.02;
      tmp(0, 1) = res[frame].points[i](1) - 0.01;
      tmp(0, 2) = layer - 0.25;
      tmp = tmp * last_rotation;
      V.row(v_per_p * offset + v_per_p * i + 23) = tmp;
    }
    offset += res[frame].points.size();
    layer += 1;
  }
  
  cog = cog / offset; // Average

  // find candidate rotations
  double rot_angle;
  if (cog(0, 0) > bases(0, 0)) { // falls to right, find min angle
    rot_angle = PI;
    for (int i = 0; i < offset; i++) {
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
    for (int i = 0; i < offset; i++) {
      if (bases.row(0) == P.row(i)) {continue;}
      double curr_rot = atan2(P(i, 1) - bases(0, 1), P(i, 0) - bases(0, 0));
      if (curr_rot > rot_angle) {
        rot_angle = curr_rot;
        bases.row(1) = P.row(i);
      }
    }
    rot_angle = PI - rot_angle;
  }

  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity(3, 3); // construct rotation matrix
  rotation(0, 0) = cos(rot_angle);
  rotation(0, 1) = sin(rot_angle);
  rotation(1, 0) = -sin(rot_angle);
  rotation(1, 1) = cos(rot_angle);

  bases = bases * rotation; // rotate base

  assert(abs(bases(0,1) - bases(1,1)) < 0.001); // extract translation
  double translate_base = bases(0, 1);

  Eigen::MatrixXd translation = Eigen::MatrixXd::Zero(num_vertices, 3); // translation for all points
  translation.col(1).setOnes();
  translation = translation * (-translate_base);

  last_rotation *= rotation; // update rotation
  last_translation = -translate_base;

  frame = (frame + 1) % ress[0].size(); // Increment frame

  V = (V * rotation) + translation;

  // Floor vertices
  V.conservativeResize(V.rows() + 8, Eigen::NoChange);
  V.bottomRows(8) << -100, 0, layer + 1,
                     100, 0, layer + 1,
                     100, 0, -1,
                     -100, 0, -1,
                     -100, -0.2, layer + 1,
                     100, -0.2, layer + 1,
                     100, -0.2, -1,
                     -100, -0.2, -1;

  return V;
}

Eigen::MatrixXd MultiAnimation::full_mesh_vertices_no_floor() {
  frame = frame % ress[0].size(); // properly bound current frame

  int v_per_p = 24;
  int num_vertices = 0;
  for (auto gm : gms) {
    num_vertices += gm.points.size();
  }

  num_vertices *= v_per_p;

  Eigen::MatrixXd V = Eigen::MatrixXd::Zero(num_vertices, 3);

  Eigen::MatrixXd tmp = Eigen::MatrixXd::Zero(1, 3);

  int layer = 0;
  int offset = 0;
  for (auto res : ress) {
    for (int i = 0; i < res[frame].points.size(); i++)
    {
      // create the points

      // 4 upper layer points
      tmp(0, 0) = res[frame].points[i](0) - 0.01;
      tmp(0, 1) = res[frame].points[i](1) - 0.01;
      tmp(0, 2) = layer + 0.25;
      V.row(v_per_p * offset + v_per_p * i) = tmp;

      tmp(0, 0) = res[frame].points[i](0) + 0.01;
      tmp(0, 1) = res[frame].points[i](1) - 0.01;
      tmp(0, 2) = layer + 0.25;
      V.row(v_per_p * offset + v_per_p * i + 1) = tmp;

      tmp(0, 0) = res[frame].points[i](0) + 0.01;
      tmp(0, 1) = res[frame].points[i](1) + 0.01;
      tmp(0, 2) = layer + 0.25;
      V.row(v_per_p * offset + v_per_p * i + 2) = tmp;

      tmp(0, 0) = res[frame].points[i](0) - 0.01;
      tmp(0, 1) = res[frame].points[i](1) + 0.01;
      tmp(0, 2) = layer + 0.25;
      V.row(v_per_p * offset + v_per_p * i + 3) = tmp;

      // 4 lower layer points
      tmp(0, 0) = res[frame].points[i](0) - 0.01;
      tmp(0, 1) = res[frame].points[i](1) - 0.01;
      tmp(0, 2) = layer - 0.25;
      V.row(v_per_p * offset + v_per_p * i + 4) = tmp;

      tmp(0, 0) = res[frame].points[i](0) + 0.01;
      tmp(0, 1) = res[frame].points[i](1) - 0.01;
      tmp(0, 2) = layer - 0.25;
      V.row(v_per_p * offset + v_per_p * i + 5) = tmp;

      tmp(0, 0) = res[frame].points[i](0) + 0.01;
      tmp(0, 1) = res[frame].points[i](1) + 0.01;
      tmp(0, 2) = layer - 0.25;
      V.row(v_per_p * offset + v_per_p * i + 6) = tmp;

      tmp(0, 0) = res[frame].points[i](0) - 0.01;
      tmp(0, 1) = res[frame].points[i](1) + 0.01;
      tmp(0, 2) = layer - 0.25;
      V.row(v_per_p * offset + v_per_p * i + 7) = tmp;

      // 8 upper layer crosses
      tmp(0, 0) = res[frame].points[i](0) - 0.01;
      tmp(0, 1) = res[frame].points[i](1) - 0.02;
      tmp(0, 2) = layer + 0.25;
      tmp = tmp * last_rotation;
      V.row(v_per_p * offset + v_per_p * i + 8) = tmp;

      tmp(0, 0) = res[frame].points[i](0) + 0.01;
      tmp(0, 1) = res[frame].points[i](1) - 0.02;
      tmp(0, 2) = layer + 0.25;
      tmp = tmp * last_rotation;
      V.row(v_per_p * offset + v_per_p * i + 9) = tmp;

      tmp(0, 0) = res[frame].points[i](0) + 0.02;
      tmp(0, 1) = res[frame].points[i](1) - 0.01;
      tmp(0, 2) = layer + 0.25;
      tmp = tmp * last_rotation;
      V.row(v_per_p * offset + v_per_p * i + 10) = tmp;

      tmp(0, 0) = res[frame].points[i](0) + 0.02;
      tmp(0, 1) = res[frame].points[i](1) + 0.01;
      tmp(0, 2) = layer + 0.25;
      V.row(v_per_p * offset + v_per_p * i + 11) = tmp;

      tmp(0, 0) = res[frame].points[i](0) + 0.01;
      tmp(0, 1) = res[frame].points[i](1) + 0.02;
      tmp(0, 2) = layer + 0.25;
      V.row(v_per_p * offset + v_per_p * i + 12) = tmp;

      tmp(0, 0) = res[frame].points[i](0) - 0.01;
      tmp(0, 1) = res[frame].points[i](1) + 0.02;
      tmp(0, 2) = layer + 0.25;
      V.row(v_per_p * offset + v_per_p * i + 13) = tmp;

      tmp(0, 0) = res[frame].points[i](0) - 0.02;
      tmp(0, 1) = res[frame].points[i](1) + 0.01;
      tmp(0, 2) = layer + 0.25;
      V.row(v_per_p * offset + v_per_p * i + 14) = tmp;

      tmp(0, 0) = res[frame].points[i](0) - 0.02;
      tmp(0, 1) = res[frame].points[i](1) - 0.01;
      tmp(0, 2) = layer + 0.25;
      V.row(v_per_p * offset + v_per_p * i + 15) = tmp;

      // 8 lower layer crosses
      tmp(0, 0) = res[frame].points[i](0) - 0.01;
      tmp(0, 1) = res[frame].points[i](1) - 0.02;
      tmp(0, 2) = layer - 0.25;
      V.row(v_per_p * offset + v_per_p * i + 16) = tmp;

      tmp(0, 0) = res[frame].points[i](0) + 0.01;
      tmp(0, 1) = res[frame].points[i](1) - 0.02;
      tmp(0, 2) = layer - 0.25;
      V.row(v_per_p * offset + v_per_p * i + 17) = tmp;

      tmp(0, 0) = res[frame].points[i](0) + 0.02;
      tmp(0, 1) = res[frame].points[i](1) - 0.01;
      tmp(0, 2) = layer - 0.25;
      V.row(v_per_p * offset + v_per_p * i + 18) = tmp;

      tmp(0, 0) = res[frame].points[i](0) + 0.02;
      tmp(0, 1) = res[frame].points[i](1) + 0.01;
      tmp(0, 2) = layer - 0.25;
      V.row(v_per_p * offset + v_per_p * i + 19) = tmp;

      tmp(0, 0) = res[frame].points[i](0) + 0.01;
      tmp(0, 1) = res[frame].points[i](1) + 0.02;
      tmp(0, 2) = layer - 0.25;
      V.row(v_per_p * offset + v_per_p * i + 20) = tmp;

      tmp(0, 0) = res[frame].points[i](0) - 0.01;
      tmp(0, 1) = res[frame].points[i](1) + 0.02;
      tmp(0, 2) = layer - 0.25;
      V.row(v_per_p * offset + v_per_p * i + 21) = tmp;

      tmp(0, 0) = res[frame].points[i](0) - 0.02;
      tmp(0, 1) = res[frame].points[i](1) + 0.01;
      tmp(0, 2) = layer - 0.25;
      V.row(v_per_p * offset + v_per_p * i + 22) = tmp;

      tmp(0, 0) = res[frame].points[i](0) - 0.02;
      tmp(0, 1) = res[frame].points[i](1) - 0.01;
      tmp(0, 2) = layer - 0.25;
      V.row(v_per_p * offset + v_per_p * i + 23) = tmp;
    }
    offset += res[frame].points.size();
    layer += 1;
  }

  frame = (frame + 1) % ress[0].size(); // Increment frame

  // Dummy floor
  V.conservativeResize(V.rows() + 8, Eigen::NoChange);
  V.bottomRows(8) << 0, 0, 0,
                     0, 0, 0,
                     0, 0, 0,
                     0, 0, 0,
                     0, 0, 0,
                     0, 0, 0,
                     0, 0, 0,
                     0, 0, 0;

  return V;
}

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

  std::cout << frame << ": " << rot_angle << std::endl;

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

void MultiAnimation::set_targets() {
  int points_set = 0;
  int edges_set = 0;
  int layer = 0;

  Eigen::Matrix<double, 1, 3> tmp;

  target_edges = Eigen::Matrix<int, Eigen::Dynamic, 2>();
  target_colors = Eigen::Matrix<double, Eigen::Dynamic, 3>();
  target_points = Eigen::Matrix<double, Eigen::Dynamic, 3>();

  // Traced Paths
  for (int ind = 0; ind < ress.size(); ind++) { // by layer
    std::vector<GridResult> res = ress[ind];
    std::vector<int> res_traces = to_trace[ind];
    
    for (int i = 0; i < res_traces.size(); i++)
    { // ith traced path
      target_edges.conservativeResize(target_edges.rows() + res.size() - 1, Eigen::NoChange);
      target_colors.conservativeResize(target_colors.rows() + res.size() - 1, Eigen::NoChange);
      target_points.conservativeResize(target_points.rows() + res.size(), Eigen::NoChange);

      for (int j = 0; j < res.size(); j++)
      { // at time j
        tmp(0, 0) = res[j].points[res_traces[i]](0);
        tmp(0, 1) = res[j].points[res_traces[i]](1);
        tmp(0, 2) = layer;
        target_points.row(points_set) = tmp; // Apply rotation
        if (j > 0) {
          target_edges(edges_set, 0) = points_set - 1;
          target_edges(edges_set, 1) = points_set;
          target_colors(edges_set, 0) = 0;
          target_colors(edges_set, 1) = 0;
          target_colors(edges_set, 2) = 1;
          edges_set++;
        }
        points_set++;
      }
    }
    layer += 1;
  }

  // Target paths
  layer = 0;
  for (auto res_target_paths : target_paths) {
    for (auto path : res_target_paths)
    {
      bool first = true;
      target_edges.conservativeResize(target_edges.rows() + path.size() - 1, Eigen::NoChange);
      target_colors.conservativeResize(target_colors.rows() + path.size() - 1, Eigen::NoChange);
      target_points.conservativeResize(target_points.rows() + path.size(), Eigen::NoChange);
      for (auto point : path)
      {
        tmp(0, 0) = point[0];
        tmp(0, 1) = point[1];
        tmp(0, 2) = layer;
        target_points.row(points_set) = tmp; // Apply rotation
        if (!first) {
          target_edges(edges_set, 0) = points_set - 1;
          target_edges(edges_set, 1) = points_set;
          target_colors(edges_set, 0) = 1;
          target_colors(edges_set, 1) = 0;
          target_colors(edges_set, 2) = 0;
          edges_set++;
        } else {
          first = false;
        }
        points_set++;
      }
    }
    layer += 1;
  } 
}

void MultiAnimation::animate_mesh()
{
  // Initialize viewer
  igl::opengl::glfw::Viewer viewer;
  viewer.data().point_size = 20;
  viewer.data().set_face_based(true);
  viewer.core().orthographic = true;
  viewer.core().toggle(viewer.data().show_lines);
  viewer.core().is_animating = true;
  viewer.core().background_color.setOnes();

  Eigen::MatrixXd translation = Eigen::MatrixXd::Zero(target_points.rows(), 3); // translation for target points
  translation.col(1).setOnes();

  int curr_in_frame = 0;

  // Animation Callback
  viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer &) -> bool
  {
    curr_in_frame++;
    if (curr_in_frame == rate)
    { // slow controls frame rate
      // Get mesh

      // auto V = full_mesh_vertices_no_floor();
      // auto V = full_mesh_vertices();

      // draw mesh
      if (draw_ground) {
        viewer.data().set_mesh(full_mesh_vertices(), F);
        viewer.data().set_colors(C);
        viewer.data().set_edges(target_points * last_rotation + translation * last_translation, target_edges, target_colors);
      } else {
        viewer.data().set_mesh(full_mesh_vertices_no_floor(), F);
        viewer.data().set_colors(C);
        viewer.data().set_edges(target_points, target_edges, target_colors);
      }
      curr_in_frame = 0;
    }

    return false;
  };

  viewer.launch();
}