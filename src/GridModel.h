#ifndef GridModel_h
#define GridModel_h

#include <vector>
#include <set>
#include <iostream>

#ifdef _WIN32
#define EIGEN_DONT_ALIGN_STATICALLY
#define EXPORT_DLL_COMMAND __declspec(dllexport)
#else
#define EXPORT_DLL_COMMAND
#endif

#include <Eigen/Dense>

enum EXPORT_DLL_COMMAND CellType
{
	RIGID = 0,
	SHEAR = 1,
	VOID = 2,
	BROKEN = 3
};

// Representing a cell in the grid by the indizes
// of its wertices and specified CellType.
struct EXPORT_DLL_COMMAND GridCell
{
public:
	Eigen::Vector4i vertices;
	CellType type;

	GridCell(const int a, const int b, const int c, const int d, CellType t);
	GridCell(const GridCell &other);

	GridCell(const int a, const int b, const int c, const int d, const int t);
};

bool operator==(const GridCell &lhs, const GridCell &rhs);

// Struct representing the optimization result
// for one frame.
struct EXPORT_DLL_COMMAND GridResult
{
	typedef Eigen::Vector2d Point;
	std::vector<Point> points;
	double objError;
	double constrError;
};

// Class representing a problem instance
class EXPORT_DLL_COMMAND GridModel
{
	typedef Eigen::Vector2d Point;
	typedef std::pair<int, int> Edge;

public:
	std::vector<Point> points;
	std::vector<GridCell> cells;

	std::vector<int> anchors;

	std::vector<int> inputs;
	std::vector<std::vector<Point>> inputPaths;

	std::vector<std::vector<Point>> targetPaths;
	std::vector<int> targets;

	std::vector<std::vector<std::pair<GridCell, std::set<Edge>>>> constraintGraph;

	GridModel(const GridModel &other)
	{
		for (Point point : other.points)
			points.push_back(Point(point));

		for (size_t i = 0; i < other.cells.size(); i++)
		{
			GridCell cell = other.cells[i];
			cells.push_back(GridCell(cell));
		}

		anchors = std::vector<int>(other.anchors);

		inputs = std::vector<int>(other.inputs);

		for (std::vector<Point> path : other.inputPaths)
		{
			inputPaths.push_back(path);
		}

		targets = std::vector<int>(other.targets);

		for (std::vector<Point> path : other.targetPaths)
		{
			targetPaths.push_back(path);
		}

		for (std::vector<std::pair<GridCell, std::set<Edge>>> component : other.constraintGraph)
		{
			constraintGraph.push_back(component);
		}
	}

	GridModel()
	{
	}

	void loadFromFile(std::string fname);
	void generateConstraintGraph();
	void mergeComponents();
	void splitComponents();

private:
	void addConstraints(std::vector<std::pair<GridCell, std::set<Edge>>> unlinkedConstraints);
	std::vector<std::vector<std::pair<GridCell, std::set<GridModel::Edge>>>> findContainingComponents(std::pair<GridCell, std::set<Edge>> constraint, std::vector<std::vector<std::pair<GridCell, std::set<Edge>>>> graph);
};

EXPORT_DLL_COMMAND
std::vector<GridResult>
optimize(const GridModel &model, std::string pointDirectory = "");

EXPORT_DLL_COMMAND
Eigen::Vector2d *
buildAndOptimize(Eigen::Vector2d *points, int numPoints, int numCells,
				 Eigen::Vector4i *cellsVertices, int *cellsTypes,
				 int *anchors, int numAnchors,
				 int *inputs, int numInputs, int *inputPathLenghts, Eigen::Vector2d *inputPaths,
				 int *targets, int numTargets, int *targetPathLenghths, Eigen::Vector2d *targetPaths);

class EXPORT_DLL_COMMAND SimuAn
{

public:
	GridModel best_model;
	double least_error;
	std::vector<GridResult> best_res;

	void simulatedAnnealing(GridModel &gm, double coolingFactor = 0.99, double startChance = 0.25);
};

#endif /* GridModel_h */
