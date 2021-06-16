#include <igl/opengl/glfw/Viewer.h>
#include "GridModel.h"

#include <fstream>

bool disable_keyboard (igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier) {
  return true;
}

std::tuple<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXd> animate_path (std::vector<GridResult> ret, std::vector<int> paths, Eigen::MatrixXi grid_edges) {
  static int counter = 0;

  // Initialize Points
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(ret[counter].points.size() + paths.size() * ret.size(), 3);

  // Current Grid
  for (int i = 0; i < ret[counter].points.size(); i++) {
    P(i,0) = ret[counter].points[i](0);
    P(i,1) = ret[counter].points[i](1);
    P(i,2) = 0;
  }

  // Path
  for (int i = 0; i < paths.size(); i++) { // ith input path
    for (int j = 0; j < ret.size(); j++) { // at time j
      P(ret[counter].points.size() + i * ret.size() + j, 0) = ret[j].points[paths[i]](0);
      P(ret[counter].points.size() + i * ret.size() + j, 1) = ret[j].points[paths[i]](1);
      P(ret[counter].points.size() + i * ret.size() + j, 2) = 0;
    }
  }

  // Initialize Edges
  Eigen::MatrixXi E = Eigen::MatrixXi::Zero(grid_edges.rows() + paths.size() * (ret.size() - 1), 2);

  // Grid Edges
  E.topRows(grid_edges.rows()) = grid_edges;

  // Path Edges
  for (int i = 0; i < paths.size(); i++) { // ith input path
    for (int j = 0; j < ret.size() - 1; j++) { // edges between time j and j+1
      E(grid_edges.rows() + i * ret.size() + j, 0) = ret[counter].points.size() + i * ret.size() + j;
      E(grid_edges.rows() + i * ret.size() + j, 1) = ret[counter].points.size() + i * ret.size() + j + 1;
    }
  }

  // Initialize Colors
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(grid_edges.rows() + paths.size() * (ret.size() - 1), 3);
  C.bottomLeftCorner(paths.size() * (ret.size() - 1), 1).setOnes();

  counter = (counter + 1) % ret.size();
  
  return std::make_tuple(P, E, C);
}

int main(int argc, char *argv[])
{

  //std::cout << "loading" << std::endl;
  GridModel gm;
  gm.loadFromFile("../cells.txt");
  //std::cout << "loaded" << std::endl;
  auto ret = optimize(gm, "../points/");

  // Initialize viewer
  igl::opengl::glfw::Viewer viewer;
  viewer.data().point_size = 20;
  viewer.data().set_face_based(true);
  viewer.core().orthographic = true;
  viewer.core().toggle(viewer.data().show_lines);
  viewer.core().is_animating = true;
  viewer.core().background_color.setOnes();

  // Original Points
  Eigen::MatrixXd OP = Eigen::MatrixXd::Zero(gm.points.size(),3);
  for (int i = 0; i < gm.points.size(); i++) {
    OP(i,0) = gm.points[i](0);
    OP(i,1) = gm.points[i](1);
    OP(i,2) = 0;
  }
  //std::cout << OP << std::endl;

  // Grid Edges
  int total_edges = 0;
  for (int i = 0; i < gm.cells.size(); i++) {
    total_edges += 4;
    if (gm.cells[i].type == RIGID) {
      total_edges += 2;
    }
  }
  Eigen::MatrixXi E = Eigen::MatrixXi::Zero(total_edges,2);

  int index = 0;
  for (int i = 0; i < gm.cells.size(); i++) {
    E(index,0) = gm.cells[i].vertices(0);
    E(index,1) = gm.cells[i].vertices(1);
    index++;
    E(index,0) = gm.cells[i].vertices(1);
    E(index,1) = gm.cells[i].vertices(2);
    index++;
    E(index,0) = gm.cells[i].vertices(2);
    E(index,1) = gm.cells[i].vertices(3);
    index++;
    E(index,0) = gm.cells[i].vertices(3);
    E(index,1) = gm.cells[i].vertices(0);
    index++;
    if (gm.cells[i].type == RIGID) {
      E(index,0) = gm.cells[i].vertices(1);
      E(index,1) = gm.cells[i].vertices(3);
      index++;
      E(index,0) = gm.cells[i].vertices(0);
      E(index,1) = gm.cells[i].vertices(2);
      index++;
    }
  }
  //std::cout << E << std::endl;

  // Eigen::MatrixXd P1 = Eigen::MatrixXd::Zero(ret[1].points.size(),3);
  // for (int i = 0; i < ret[1].points.size(); i++) {
  //   P1(i,0) = ret[1].points[i](0);
  //   P1(i,1) = ret[1].points[i](1);
  //   P1(i,2) = 0;
  // }
  // std::cout << P1 << std::endl;


  // viewer.data().add_points(P1, Eigen::RowVector3d(1,0,0));

  // for (int i = 0; i < gm.cells.size(); i++) {
  //   viewer.data().add_edges(P1.row(gm.cells[i].vertices(0)), P1.row(gm.cells[i].vertices(1)), Eigen::RowVector3d(1,0,0));
  //   viewer.data().add_edges(P1.row(gm.cells[i].vertices(1)), P1.row(gm.cells[i].vertices(2)), Eigen::RowVector3d(1,0,0));
  //   viewer.data().add_edges(P1.row(gm.cells[i].vertices(2)), P1.row(gm.cells[i].vertices(3)), Eigen::RowVector3d(1,0,0));
  //   viewer.data().add_edges(P1.row(gm.cells[i].vertices(3)), P1.row(gm.cells[i].vertices(0)), Eigen::RowVector3d(1,0,0));
  //   if (gm.cells[i].type == RIGID) {
  //     viewer.data().add_edges(P1.row(gm.cells[i].vertices(1)), P1.row(gm.cells[i].vertices(3)), Eigen::RowVector3d(1,0,0));
  //     viewer.data().add_edges(P1.row(gm.cells[i].vertices(0)), P1.row(gm.cells[i].vertices(2)), Eigen::RowVector3d(1,0,0));
  //   }
  // }

  // slow controls frame rate
  int slow = 0;

  // Animation Callback
  viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer & )->bool
  {
    slow++;
    if (slow == 1) { // slow controls frame rate
      // Get points and edges to draw
      auto tup = animate_path(ret, gm.targets, E);
      auto points = std::get<0>(tup);
      auto edges = std::get<1>(tup);
      auto colors = std::get<2>(tup);
      // update points and edges
      viewer.data().set_points(points.topRows(gm.points.size()), Eigen::RowVector3d(0,0,0));
      viewer.data().set_edges(points, edges, colors);
      slow = 0;
    }
    
    return false;
  };

  viewer.launch();
}
