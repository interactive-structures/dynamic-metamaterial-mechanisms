#pragma once
#include "GridModel.h"
#include "MetaGrid.hpp"

#include <string>
#include <vector>


using namespace std;


std::string toString(std::pair<int, int> pair);
std::string toString(std::vector<int> indices);



std::string toString(GridModel& model);

std::string toString(Point& point);
std::string toString(std::vector<Point> points);
std::string toString(std::vector<std::vector<Point>> points);

std::string toString(GridCell& cell);
std::string toString(std::vector<GridCell> cells);

std::string toString(std::vector<std::vector<std::pair<GridCell, std::set<Edge>>>> constraintGraph);




std::string toString(MetaGrid& grid);

std::string toString(Cell& cell);
std::string toString(std::vector<Cell> cells);

std::string toString(Edge& edge);
std::string toString(std::vector<Edge> edges);
std::string toString(RelEdge relEdge);
std::string toString(std::vector<std::vector<RelEdge>> edgeGraph);

std::string toString(std::vector<std::pair<int, std::vector<Point>>> constrained);
std::string toString(std::vector<Eigen::Matrix<double, 2, -1>> vertexTransformations);
