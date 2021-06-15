#include <igl/opengl/glfw/Viewer.h>
#include "GridModel.h"

#include <fstream>

bool disable_keyboard (igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier) {
  return true;
}

int main(int argc, char *argv[])
{
  std::cout << "loading" << std::endl;
  GridModel gm;
  gm.loadFromFile("../cells.txt");
  std::cout << "loaded" << std::endl;
  auto ret = optimize(gm, "../points/");

  // Plot the mesh
  igl::opengl::glfw::Viewer viewer;

  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(gm.points.size(),3);
  for (int i = 0; i < gm.points.size(); i++) {
    P(i,0) = gm.points[i](0);
    P(i,1) = gm.points[i](1);
    P(i,2) = 0;
  }
  //std::cout << P << std::endl;

  viewer.data().add_points(P, Eigen::RowVector3d(0,0,0));
  viewer.data().point_size = 20;

  for (int i = 0; i < gm.cells.size(); i++) {
    viewer.data().add_edges(P.row(gm.cells[i].vertices(0)), P.row(gm.cells[i].vertices(1)), Eigen::RowVector3d(0,0,0));
    viewer.data().add_edges(P.row(gm.cells[i].vertices(1)), P.row(gm.cells[i].vertices(2)), Eigen::RowVector3d(0,0,0));
    viewer.data().add_edges(P.row(gm.cells[i].vertices(2)), P.row(gm.cells[i].vertices(3)), Eigen::RowVector3d(0,0,0));
    viewer.data().add_edges(P.row(gm.cells[i].vertices(3)), P.row(gm.cells[i].vertices(0)), Eigen::RowVector3d(0,0,0));
    if (gm.cells[i].type == RIGID) {
      viewer.data().add_edges(P.row(gm.cells[i].vertices(1)), P.row(gm.cells[i].vertices(3)), Eigen::RowVector3d(0,0,0));
      viewer.data().add_edges(P.row(gm.cells[i].vertices(0)), P.row(gm.cells[i].vertices(2)), Eigen::RowVector3d(0,0,0));
    }
  }

  viewer.data().set_face_based(true);
  viewer.core().orthographic = true;
  viewer.core().toggle(viewer.data().show_lines);
  viewer.launch();
}
