#include "StringConverter.h"


std::string toString(GridModel& model)
{
	std::ostringstream out;
	out << "---------------------------------------------\n";
	out << "GridModel: \n";
	out << "---\n";

	out << "Cells: \n" << toString(model.cells) << std::endl;
	out << "Anchors: \n" << toString(model.anchors) << std::endl;
	out << "Target vertices: \n" << toString(model.targets) << std::endl;
	out << "Target paths: \n" << toString(model.targetPaths) << std::endl;
	out << "Input vertices: \n" << toString(model.inputs) << std::endl;
	out << "Input paths: \n" << toString(model.inputPaths) << std::endl;
	out << "Constraint graph: \n" << toStringConstraintGraph(model) << std::endl;
	
	out << "---------------------------------------------\n";

	return out.str();
}

std::string toString(std::pair<int, int> pair)
{
	std::ostringstream out;
	out << "(" << pair.first << "," << pair.second << ")";
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
	//out << "Points: \n";

	for (int i = 0; i < points.size(); i++)
		out << "  [" << i << "]: " << toString(points[i]) << "\n";

	//out << "\n";
	return out.str();
}

std::string toString(std::vector<std::vector<Point>> paths)
{
	std::ostringstream out;
	//out << "Paths: \n";

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
	//out << "GridCells: \n";

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

std::string toStringConstraintGraph(GridModel& model)
{
	auto constraintGraph = model.constraintGraph;
	std::ostringstream out;

	for (int i = 0; i < constraintGraph.size(); i++)
	{
		out << "[" << i << "] Connected component: \n";
		for (int j = 0; j < constraintGraph[i].size(); j++)
		{
			auto pair = constraintGraph[i][j];
			out << "  [" << j << "]: \n";
			out << "    Cell: " << toString(pair.first) << "\n";
			
			out << "    Constrained edges: ";

			//for (int k = 0; k < pair.second.size(); k++)
			//	out << "    [" << k << "]: " << toString(pair.second[k]) << "\n";

			for (auto edge : pair.second) 
				out << toString(edge) << ", ";

			out << "\n";
		}
	}

	return out.str();
}

//std::string printConstraintGraph(GridModel gm)
//{
//	auto cG = gm.constraintGraph;
//	std::cout << std::endl
//		<< "Constraint Graph [" << std::endl;
//	int compCount = 0;
//
//	for (auto comp : cG)
//	{
//		std::cout << "Component " << compCount << std::endl;
//		for (auto constraint : comp)
//		{
//			std::cout << "{";
//			std::cout << "Cell: " << constraint.first.vertices[0] << ", " << constraint.first.vertices[1];
//			std::cout << ", " << constraint.first.vertices[2] << ", " << constraint.first.vertices[3] << ". Constraints:";
//			for (auto edge : constraint.second)
//			{
//				std::cout << " (" << edge.first << "," << edge.second << ")";
//			}
//			std::cout << "} ";
//		}
//		compCount++;
//		std::cout << std::endl;
//	}
//	std::cout << "]" << std::endl;
//}


std::string toString(MetaGrid& grid)
{
	//Point shift;
	//std::vector<int> anchors;
	//std::vector<Point> anchorPositions;
	//std::vector<Point> vertices;
	//std::vector<Cell> cells;
	//std::vector<Edge> edges;
	//std::vector<std::pair<int, std::vector<Point>>> constrained;
	//std::vector<Eigen::Matrix<double, 2, -1>> vertexTransformations;

	//----

	//std::vector<std::vector<RelEdge>> edgeGraph;

	std::ostringstream out;
	out << "---------------------------------------------\n";
	out << "MetaGrid: \n";
	out << "---\n";

	out << "Shift: \n" << toString(grid.shift) << std::endl;
	out << "Anchor vertices: \n" << toString(grid.anchors) << std::endl;
	out << "Anchor positions: \n" << toString(grid.anchorPositions) << std::endl;
	out << "Vertex positions: \n" << toString(grid.vertices) << std::endl;

	out << "Cells: \n" << toString(grid.cells) << std::endl;
	out << "Edges: \n" << toString(grid.edges) << std::endl;
	out << "Constrains: \n" << toString(grid.constrained) << std::endl;
	out << "VertexTransformations: \n" << toString(grid.vertexTransformations) << std::endl;
	out << "EdgeGraph: \n" << toString(grid.edgeGraph) << std::endl;

	out << "---------------------------------------------\n";

	return out.str();
}


std::string toString(Cell& cell) 
{
	std::ostringstream out;

	out << enumString[cell.type] << ";  vertices: [" << cell.V[0] << ", " << cell.V[1] << ", " << cell.V[2] << ", " << cell.V[3] << "]; ";
	out << "edges: [" << toString(cell.edges[0]) << ", " << toString(cell.edges[1]) << ", " << toString(cell.edges[2]) << ", " << toString(cell.edges[3]) << "]";

	return out.str();
}

std::string toString(std::vector<Cell> cells)
{
	std::ostringstream out;

	for (int i = 0; i < cells.size(); i++)
		out << "  [" << i << "]: " << toString(cells[i]) << "\n";

	return out.str();
}

std::string toString(Edge& edge)
{
	std::ostringstream out;
	out << edge.i << ", " << edge.j << "; flag=" << edge.flag;
	return out.str();
}

std::string toString(std::vector<Edge> edges)
{
	std::ostringstream out;

	for (int i = 0; i < edges.size(); i++)
		out << "  [" << i << "]: " << toString(edges[i]) << "\n";

	return out.str();
}

std::string toString(RelEdge relEdge)
{
	std::ostringstream out;
	out << "index: " << relEdge.i << ", cell index: " << relEdge.cell_index << ", relation type: " << relEdge.type;
	return out.str();
}

std::string toString(std::vector<std::vector<RelEdge>> edgeGraph)
{
	std::ostringstream out;

	for (int i = 0; i < edgeGraph.size(); i++)
	{
		out << "[" << i << "]: \n";
		for (int j = 0; j < edgeGraph[i].size(); j++)
			out << " " << toString(edgeGraph[i][j]) << "\n";
	}

	return out.str();
}

std::string toString(std::vector<std::pair<int, std::vector<Point>>> constrained)
{
	std::ostringstream out;

	for (int i = 0; i < constrained.size(); i++)
	{
		auto pair = constrained[i];
		out << "  [" << i << "]: " << "vertex: " << pair.first << "\n  points: \n" << toString(pair.second);
	}

	return out.str();
}

std::string toString(std::vector<Eigen::Matrix<double, 2, -1>> vertexTransformations)
{
	std::ostringstream out;

	for (int i = 0; i < vertexTransformations.size(); i++)
	{
		out << "[" << i << "]: \n" << vertexTransformations[i] << "\n";
	}

	return out.str();
}
