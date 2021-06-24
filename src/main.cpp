#include <igl/opengl/glfw/Viewer.h>
#include "GridModel.h"
#include "cppopt/cppOpt.h"
#include <fstream>
#include <tuple>

// Used to disable inputs to control libigl viewer. Safe to ignore for now.
bool disable_keyboard(igl::opengl::glfw::Viewer &viewer, unsigned char key, int modifier)
{
  return true;
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

// Returns matrix representing edges to visualize a grid. Note that shared edges are represented twice, though this is fine for our purposes.
Eigen::MatrixXi get_edge_matrix(GridModel gm)
{
  int total_edges = 0;
  for (int i = 0; i < gm.cells.size(); i++)
  {
    total_edges += 4;
    if (gm.cells[i].type == RIGID)
    {
      total_edges += 2;
    }
  }
  Eigen::MatrixXi E = Eigen::MatrixXi::Zero(total_edges, 2);

  int index = 0;
  for (int i = 0; i < gm.cells.size(); i++)
  {
    // Add four boundary edges
    E(index, 0) = gm.cells[i].vertices(0);
    E(index, 1) = gm.cells[i].vertices(1);
    index++;
    E(index, 0) = gm.cells[i].vertices(1);
    E(index, 1) = gm.cells[i].vertices(2);
    index++;
    E(index, 0) = gm.cells[i].vertices(2);
    E(index, 1) = gm.cells[i].vertices(3);
    index++;
    E(index, 0) = gm.cells[i].vertices(3);
    E(index, 1) = gm.cells[i].vertices(0);
    index++;
    if (gm.cells[i].type == RIGID)
    { // Diagonal edges
      E(index, 0) = gm.cells[i].vertices(1);
      E(index, 1) = gm.cells[i].vertices(3);
      index++;
      E(index, 0) = gm.cells[i].vertices(0);
      E(index, 1) = gm.cells[i].vertices(2);
      index++;
    }
  }

  return E;
}

/** Inputs: vector of GridResults representing state of grid at each timestep, original GridModel, matrix of edges on the grid
 *  Outputs: a tuple of (points needed to plot, edges to plot, colors of points, colors of edges)
 *  Note that only the first gm.points.size() rows of points needed to plot are actually the points to plot, the remainder are used to plot the edges of the target path.
 */
std::tuple<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXd, Eigen::MatrixXd> animate_path(std::vector<GridResult> ret, GridModel gm, Eigen::MatrixXi grid_edges)
{
  static int counter = 0; // Tracks current frame to display

  // Initialize Points
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(ret[counter].points.size() + gm.targets.size() * ret.size(), 3);

  // Points on grid
  for (int i = 0; i < ret[counter].points.size(); i++)
  {
    P(i, 0) = ret[counter].points[i](0);
    P(i, 1) = ret[counter].points[i](1);
    P(i, 2) = 0;
  }

  // Points on path
  for (int i = 0; i < gm.targets.size(); i++)
  { // ith input path
    for (int j = 0; j < ret.size(); j++)
    { // at time j
      P(ret[counter].points.size() + i * ret.size() + j, 0) = ret[j].points[gm.targets[i]](0);
      P(ret[counter].points.size() + i * ret.size() + j, 1) = ret[j].points[gm.targets[i]](1);
      P(ret[counter].points.size() + i * ret.size() + j, 2) = 0;
    }
  }

  // Initialize Edges
  Eigen::MatrixXi E = Eigen::MatrixXi::Zero(grid_edges.rows() + gm.targets.size() * (ret.size() - 1), 2);

  // Edges on grid
  E.topRows(grid_edges.rows()) = grid_edges;

  // Edges on path
  for (int i = 0; i < gm.targets.size(); i++)
  { // ith input path
    for (int j = 0; j < ret.size() - 1; j++)
    { // edges between time j and j+1
      E(grid_edges.rows() + i * ret.size() + j, 0) = ret[counter].points.size() + i * ret.size() + j;
      E(grid_edges.rows() + i * ret.size() + j, 1) = ret[counter].points.size() + i * ret.size() + j + 1;
    }
  }

  // Point Colors
  Eigen::MatrixXd PC = Eigen::MatrixXd::Zero(ret[counter].points.size(), 3);
  for (int i : gm.anchors)
  {
    PC.row(i) = Eigen::RowVector3d(1, 0, 0);
  }

  // Edge Colors
  Eigen::MatrixXd EC = Eigen::MatrixXd::Zero(grid_edges.rows() + gm.targets.size() * (ret.size() - 1), 3);
  EC.bottomLeftCorner(gm.targets.size() * (ret.size() - 1), 1).setOnes();

  counter = (counter + 1) % ret.size(); // Increment frame

  return std::make_tuple(P, E, PC, EC);
}

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
  gm.generateConstraintGraph();

  //std::cout << gm.constraintGraph.size() << std::endl;

  printConstraintGraph(gm);
  // gm.splitComponents();
  // printConstraintGraph(gm);
  // gm.mergeComponents();
  // printConstraintGraph(gm);
  // gm.mergeComponents();
  // printConstraintGraph(gm);
  // gm.splitComponents();
  // printConstraintGraph(gm);

  //std::cout << "loaded" << std::endl;

  // auto ret = optimize(gm, "../points/");
  srand(time(NULL)); // Initialize rng

  auto toOptimize = [&](cppOpt::OptCalculation<double> &optCalculation)
  {
    double x = optCalculation.get_parameter("x");
    double error = 0;
    if (rand() % 2 == 0)
    {
      gm.splitComponents();
    }
    else
    {
      gm.mergeComponents();
    }
    auto ret2 = optimize(gm, "../points/");
    for (auto re : ret2)
    {
      error += re.constrError;
      error += re.objError;
    }
    optCalculation.result = error;
  };

  cppOpt::OptBoundaries<double> optBoundaries;
  optBoundaries.add_boundary({-1, 1, "x"});

  //number of calculations
  unsigned int maxCalculations = 30;

  //we want to find the minimum
  cppOpt::OptTarget optTarget = cppOpt::OptTarget::MINIMIZE;

  //how fast the simulated annealing algorithm slows down
  //http://en.wikipedia.org/wiki/Simulated_annealing
  double coolingFactor = 0.99;

  //the chance in the beginning to follow bad solutions
  double startChance = 0.25;

  //define your coordinator
  cppOpt::OptCoordinator<double, false> coordinator(
      maxCalculations,
      toOptimize,
      optTarget,
      0);

  //add simulated annealing as child
  coordinator.add_child(make_unique<cppOpt::OptSimulatedAnnealing<double>>(
      optBoundaries,
      coolingFactor,
      startChance));

  //let's go
  coordinator.run_optimisation();

  //print result
  cppOpt::OptCalculation<double> best = coordinator.get_best_calculation();
  cout << best.to_string_header() << endl;
  cout << best.to_string_values() << endl;

  auto ret = optimize(gm, "../points/");
  // auto angles = get_angles(ret, gm);
  // std::cout << "Frame 0 angles: ";
  // for (double angle : angles[0]) {
  //   std::cout << angle << " ";
  // }
  // std::cout << std::endl;

  // Initialize viewer
  igl::opengl::glfw::Viewer viewer;
  viewer.data().point_size = 20;
  viewer.data().set_face_based(true);
  viewer.core().orthographic = true;
  viewer.core().toggle(viewer.data().show_lines);
  viewer.core().is_animating = true;
  viewer.core().background_color.setOnes();

  // Original Points
  // Eigen::MatrixXd OP = Eigen::MatrixXd::Zero(gm.points.size(),3);
  // for (int i = 0; i < gm.points.size(); i++) {
  //   OP(i,0) = gm.points[i](0);
  //   OP(i,1) = gm.points[i](1);
  //   OP(i,2) = 0;
  // }
  //std::cout << OP << std::endl;

  Eigen::MatrixXi E = get_edge_matrix(gm);
  //std::cout << E << std::endl;

  // slow controls frame rate
  int slow = 0;

  // Animation Callback
  viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer &) -> bool
  {
    slow++;
    if (slow == 1)
    { // slow controls frame rate
      // Get points and edges to draw
      auto tup = animate_path(ret, gm, E);
      auto points = std::get<0>(tup);
      auto edges = std::get<1>(tup);
      auto pointColors = std::get<2>(tup);
      auto edgeColors = std::get<3>(tup);
      // update points and edges
      viewer.data().set_points(points.topRows(gm.points.size()), pointColors);
      viewer.data().set_edges(points, edges, edgeColors);
      slow = 0;
    }

    return false;
  };

  viewer.launch();
}
