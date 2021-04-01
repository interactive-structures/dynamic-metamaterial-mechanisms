#ifndef GridModel_h
#define GridModel_h

#include <vector>

#ifdef _WIN32
#define EIGEN_DONT_ALIGN_STATICALLY
#endif

#include <Eigen/Dense>

enum CellType
{
    RIGID,
    SHEAR,
    VOID
};

// Representing a cell in the grid by the indizes
// of its wertices and specified CellType.
struct GridCell
{
    Eigen::Vector4i vertices;
    CellType type;
    
    GridCell(const int a, const int b, const int c, const int d, CellType t);
};

// Class representing a problem instance
class GridModel
{
    typedef Eigen::Vector2d Point;
public:
    int nrows, ncols, root, driver;
    
    std::vector<Point> points;
    std::vector<GridCell> cells;
    
    std::vector<int> anchors;
    
    std::vector<int> inputs;
    std::vector<std::vector<Point>> inputPaths;
    
    std::vector<int> targets;
    std::vector<std::vector<Point>> targetPaths;
    
    void loadFromFile(std::string fname);
};


// Struct representing the optimization result
// for one frame.
struct GridResult
{
    typedef Eigen::Vector2d Point;
    std::vector<Point> points;
};

std::vector<GridResult>
optimize(const GridModel& model, std::string pointDirectory = "", bool elastic = true);

#endif /* GridModel_h */
