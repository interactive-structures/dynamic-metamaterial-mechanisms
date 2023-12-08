#include "StringConverter.h"


std::string toString(GridModel& model)
{
	std::ostringstream out;
	out << "---------------------------------------------\n";
	out << "GridModel: \n";
	out << "---\n";
	out << toString(model.cells) << std::endl;
	out << "Anchors: \n" << toString(model.anchors) << std::endl;
	out << "Target vertices: \n" << toString(model.targets) << std::endl;
	out << "Target paths: " << toString(model.targetPaths) << std::endl;
	out << "Input vertices: \n" << toString(model.inputs) << std::endl;
	out << "Input paths: " << toString(model.inputPaths) << std::endl;
	//out << "TODO Constraint graph: " << toString(model.constraintGraph) << std::endl;
	out << "---------------------------------------------\n";

	return out.str();
}

std::string toString(std::pair<int, int> edge)
{
	std::ostringstream out;
	out << "Edge vertices: " << edge.first << "," << edge.second;
	return out.str();
}

std::string toString(Point& point)
{
	std::ostringstream out;
	out << "(" << point[0] << "," << point[1] << ")";
	return out.str();
}

std::string toString(std::vector<Point> points)
{
	std::ostringstream out;
	out << "Points: \n";

	for (int i = 0; i < points.size(); i++)
		out << "  [" << i << "]: " << toString(points[i]) << "\n";

	//out << "\n";
	return out.str();
}

std::string toString(std::vector<std::vector<Point>> paths)
{
	std::ostringstream out;
	out << "Paths: \n";

	for (int i = 0; i < paths.size(); i++)
	{
		out << "[" << i << "]: ";
		out << toString(paths[i]) << "\n";
	}

	//out << "\n";
	return out.str();
}

std::string toString(GridCell& cell)
{
	std::ostringstream out;

	//std::string type;
	//switch (cell.type)
	//{
	//case 0:
	//	type = "RIGID";
	//	break;
	//case 1:
	//	type = "SHEAR";
	//	break;
	//case 2:
	//	type = "VOID";
	//	break;
	//case 4:
	//	type = "ACTIVE";
	//	break;
	//default:
	//	type = "BROKEN";
	//	break;
	//}

	out << enumString[cell.type] << ";  vertices: [" << cell.vertices[0] << ", " << cell.vertices[1] << ", " << cell.vertices[2] << ", " << cell.vertices[3] << "]";
	return out.str();
}

std::string toString(std::vector<GridCell> cells)
{
	std::ostringstream out;
	out << "GridCells: \n";

	for (int i = 0; i < cells.size(); i++)
		out << "  [" << i << "]: " << toString(cells[i]) << "\n";

	//out << "\n";
	return out.str();
}

std::string toString(std::vector<int> indices)
{
	std::ostringstream out;

	for (int i = 0; i < indices.size(); i++)
		out << "  [" << i << "]: " << indices[i] << "\n";

	//out << "\n";
	return out.str();
}

std::string toString(std::vector<std::vector<std::pair<GridCell, std::set<Edge>>>> constraintGraph)
{
	return "";
}


std::string toString(MetaGrid& grid)
{
	//Point shift;
	//std::vector<int> anchors;
	//std::vector<Point> anchorPositions;
	//std::vector<Point> vertices;

	//----
	//std::vector<Cell> cells;
	//std::vector<Edge> edges;
	//std::vector<std::pair<int, std::vector<Point>>> constrained;
	//std::vector<Eigen::Matrix<double, 2, -1>> vertexTransformations;

	//std::vector<std::vector<RelEdge>> edgeGraph;

	std::ostringstream out;
	out << "---------------------------------------------\n";
	out << "MetaGrid: \n";
	out << "---\n";
	out << "Shift: \n" << toString(grid.shift) << std::endl;
	out << "Anchor vertices: \n" << toString(grid.anchors) << std::endl;
	out << "Anchor positions: \n" << toString(grid.anchorPositions) << std::endl;
	out << "Vertex positions: \n" << toString(grid.vertices) << std::endl;


	//out << toString(model.cells) << std::endl;
	//out << "Target vertices: \n" << toString(model.targets) << std::endl;
	//out << "Target paths: " << toString(model.targetPaths) << std::endl;
	//out << "Input vertices: \n" << toString(model.inputs) << std::endl;
	//out << "Input paths: " << toString(model.inputPaths) << std::endl;
	//out << "TODO Constraint graph: " << toString(model.constraintGraph) << std::endl;
	out << "---------------------------------------------\n";

	return out.str();
}


std::string toString(Cell& cell) 
{
	return "";
}

std::string toString(std::vector<Cell> cells)
{
	return "";
}

std::string toString(Edge& edge)
{
	return "";
}

std::string toString(std::vector<Edge> edges)
{
	return "";
}

std::string toString(RelEdge relEdge)
{
	return "";
}

std::string toString(std::vector<std::vector<RelEdge>> edgeGraph)
{
	return "";
}

std::string toString(std::vector<std::pair<int, std::vector<Point>>> constrained)
{
	return "";
}

std::string toString(std::vector<Eigen::Matrix<double, 2, -1>> vertexTransformations)
{
	return "";
}
