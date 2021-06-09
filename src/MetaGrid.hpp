#ifndef MetaGrid_hpp
#define MetaGrid_hpp

#ifdef _WIN32
#define EIGEN_DONT_ALIGN_STATICALLY
#endif

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <array>
#include "GridModel.h"

enum RelationType
{
    IDENTITY,
    INVERTED,
    ROT90,
    ROT270
};

struct Edge
{
    int i, j;

    bool flag = false;
    
    int operator[](const int id) const {return id == 0 ? i : j;};
    
    bool operator<(const Edge& e) const
    {
        if(i < e.i) return true;
        else if(i == e.i && j < e.i) return true;
        
        return false;
    };
    
    bool operator==(const Edge& e) const
    {
        return i == e.i && j == e.j;
    }
    
    Edge(const int _i, const int _j)
    : i(_i), j(_j) {}
};

struct Cell
{
    CellType type;
    std::array<int, 4> V;
    std::array<std::pair<int, RelationType>, 4> edges;
    Cell(CellType _type, const int a, const int b, const int c, const int d);

    int operator[](const int i) const {return V[i];};
};

typedef Eigen::Vector2d Point;
typedef Eigen::Vector2d Vector2d;

struct RelEdge
{
    int i;
    RelationType type;
    
    RelEdge(int _i, RelationType _type)
    : i(_i), type(_type)
    {}
    
    char flag = false;
};


void savePoints(std::string fname, std::vector<Point>& pts);

class MetaGrid
{
    std::vector<std::vector<RelEdge>> edgeGraph;
    std::vector<std::vector<int>> connectedComponents(std::vector<std::vector<RelEdge>>& graph);

public:
    Point shift;
    std::vector<int> anchors;
    std::vector<Point> anchorPositions;
    
    std::vector<Cell> cells;
    std::vector<Point> vertices;
    std::vector<Edge> edges;
    std::vector<std::pair<int, std::vector<Point>>> constrained;
    std::vector<Eigen::Matrix<double, 2, -1>> vertexTransformations;
    
    MetaGrid();
    MetaGrid(const GridModel& model);
    
    void load(std::string fname);
    void setEdgeRelations();
    void setEdges();
    
    std::vector<Point> propagateDOFs(const std::vector<std::pair<int, Vector2d>>& dofs);
    std::vector<Eigen::Matrix<double, 2, -1>> propagateEdgeDOFs(const std::vector<int>& dofs);
    std::vector<Eigen::Matrix<double, 2, -1>> propagateVertexDOFs(const std::vector<Eigen::Matrix<double, 2, -1>>& edgeTransformations);

    std::vector<Point> setDOFs(const std::vector<int>& dofs, const std::vector<double>& values);
    std::vector<int> degreesOfFreedom();
};

#endif /* MetaGrid_hpp */
