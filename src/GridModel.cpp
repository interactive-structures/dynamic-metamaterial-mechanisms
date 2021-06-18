#include "GridModel.h"
#include "MetaGrid.hpp"
#include "coin/IpIpoptApplication.hpp"
#include "coin/IpSolveStatistics.hpp"
#include "MyNLP.hpp"

#include <fstream>
#include <stdlib.h>
#include <time.h>

bool operator==(const GridCell& lhs, const GridCell& rhs) {
	return lhs.type == rhs.type && lhs.vertices.isApprox(rhs.vertices);
}

namespace {
	int optimize(SmartPtr<MyNLP>& mynlp)
	{
		sort(mynlp->fixedDofs.begin(), mynlp->fixedDofs.end());
		SmartPtr<IpoptApplication> app = IpoptApplicationFactory();

		// Initialize the IpoptApplication and process the options
		ApplicationReturnStatus status;
		status = app->Initialize();

		if (status != Solve_Succeeded) {
			std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
			return (int)status;
		}

		//  app->Options()->SetNumericValue("bound_mult_reset_threshold", 100);
		//   app->Options()->SetNumericValue("bound_push", 1e-4);

		app->Options()->SetIntegerValue("print_level", 0);
		//app->Options()->SetStringValue("derivative_test", "second-order");
		status = app->OptimizeTNLP(mynlp);


		/*
		 if (status == Solve_Succeeded) {
		 // Retrieve some statistics about the solve
		 Index iter_count = app->Statistics()->IterationCount();
		 std::cout << std::endl << std::endl << "*** The problem solved in " << iter_count << " iterations!" << std::endl;

		 Number final_obj = app->Statistics()->FinalObjective();
		 std::cout << std::endl << std::endl << "*** The final value of the objective function is " << final_obj << '.' << std::endl;
		 }
		 */

		mynlp->constrId.clear();
		mynlp->constrTarget.clear();
		mynlp->fixedDofs.clear();

		return (int)status;
	}
}

GridCell::GridCell(const int a, const int b, const int c, const int d, CellType t)
	: vertices(a, b, c, d), type(t)
{
}


GridCell::GridCell(const int a, const int b, const int c, const int d, const int t)
	: vertices(a, b, c, d)
{
	switch (t)
	{
	case 0:
		type = CellType(RIGID);
		break;
	case 1:
		type = CellType(SHEAR);
		break;
	case 2:
		type = CellType(VOID);
		break;
	default:
		type = CellType(BROKEN);
		break;
	}
}

GridCell::GridCell(const GridCell &other)
{
	type = CellType(other.type);
	vertices = Eigen::Vector4i(other.vertices);
}

void GridModel::loadFromFile(const std::string fname)
{
	using namespace std;
	ifstream file(fname);

	if (!file.good())
	{
		cout << "file " << fname << " not found!" << endl;
		return;
	}
	char tmp[1024];

	file.getline(tmp, 1024);

	int nv, nc, na, constr, npath, constr2 = -1, npath2;

	file >> nv;
	file >> nc;
	file >> na;
	file >> constr;
	file >> npath;

	if (file.peek() != '\n')
	{
		file >> constr2;
		file >> npath2;
	}

	if (constr2 != -1) inputs.push_back(constr2);
	targets.push_back(constr);

	file.getline(tmp, 1024);
	file.getline(tmp, 1024);
	file.getline(tmp, 1024);

	for (int i = 0; i < nv; ++i)
	{
		double x, y;
		file >> x;
		file >> y;

		points.push_back(Point(x, y));
	}

	file.getline(tmp, 1024);
	file.getline(tmp, 1024);
	file.getline(tmp, 1024);

	for (int i = 0; i < na; ++i)
	{
		int x;
		file >> x;

		anchors.push_back(x);
	}

	// shift = vertices[anchors.front()];
	 //  anchors.pop_back();

	file.getline(tmp, 1024);
	file.getline(tmp, 1024);
	file.getline(tmp, 1024);

	for (int i = 0; i < nc; ++i)
	{
		char t;
		file >> t;

		int a, b, c, d;
		file >> a;
		file >> b;
		file >> c;
		file >> d;

		cells.push_back(GridCell(a, b, c, d, t == 's' ? SHEAR : RIGID));
	}

	file.getline(tmp, 1024);
	file.getline(tmp, 1024);
	file.getline(tmp, 1024);

	vector<Point> path;

	for (int i = 0; i < npath; ++i)
	{
		double x, y;
		file >> x;
		file >> y;

		path.push_back(Point(x, y));
	}

	targetPaths.push_back(path);
	path.clear();

	if (constr2 != -1)
	{
		file.getline(tmp, 1024);
		file.getline(tmp, 1024);
		file.getline(tmp, 1024);

		for (int i = 0; i < npath2; ++i)
		{
			double x, y;
			file >> x;
			file >> y;

			path.push_back(Point(x, y));
		}

		inputPaths.push_back(path);
	}

	file.close();
}

// Returns std::vector of all components of the given graph containing an edge also in the given constraint.
std::vector<std::vector<std::pair<GridCell, std::set<GridModel::Edge>>>> GridModel::findContainingComponents(std::pair<GridCell, std::set<Edge>> constraint, std::vector<std::vector<std::pair<GridCell, std::set<Edge>>>> graph) {
  std::vector<std::vector<std::pair<GridCell, std::set<Edge>>>> containingComponents;
	for (std::vector<std::pair<GridCell, std::set<Edge>>> graphComponent : graph) {
		for (std::pair<GridCell, std::set<Edge>> existingConstraint : graphComponent) {
			for (Edge e : constraint.second) {
				if (existingConstraint.second.find(e) != existingConstraint.second.end()) {
					containingComponents.push_back(graphComponent);
					break;
				}
			}
		}
	}
	return containingComponents;
}

void GridModel::addConstraints(std::vector<std::pair<GridCell, std::set<Edge>>> unlinkedConstraints) {
	std::vector<std::pair<GridCell, std::set<Edge>>> toVisit(unlinkedConstraints); // Initialize constraints to check

	while (toVisit.size() > 0) // Do while there remains unassigned constraints
	{
		std::pair<GridCell, std::set<Edge>> constraint = toVisit[0]; // Get next constraint
		toVisit.erase(toVisit.begin());

		std::vector<std::pair<GridCell, std::set<Edge>>> component({constraint}); // Create a new component
		std::vector<std::vector<std::pair<GridCell, std::set<Edge>>>> containingComponents = findContainingComponents(constraint, constraintGraph); // Find existing components

		if (containingComponents.size() > 0) { // Join all existing components
      for (std::vector<std::pair<GridCell, std::set<Edge>>> containingComponent : containingComponents) {
				for (std::pair<GridCell, std::set<Edge>> existingConstraint : containingComponent) { // For each existing constraint
					if (std::find(component.begin(), component.end(), existingConstraint) == component.end()) {
						component.push_back(existingConstraint); // Add only if constraint does not yet exist
					}
				}
				constraintGraph.erase(std::find(constraintGraph.begin(), constraintGraph.end(), containingComponent));
    	}
    }
    constraintGraph.push_back(component);
		//std::cout << "Component added. " << constraintGraph.size() << " total components" << std::endl;
	}
	
}

void GridModel::generateConstraintGraph() {

	// Track unassigned constraints
	std::vector<std::pair<GridCell, std::set<Edge>>> unlinkedConstraints;
	std::vector<GridCell> associatedCells;

	// Collect all constraints
  for (auto c : cells) {
		Edge e1 = std::make_pair(std::min(c.vertices[0], c.vertices[1]), std::max(c.vertices[0], c.vertices[1]));
		Edge e2 = std::make_pair(std::min(c.vertices[1], c.vertices[2]), std::max(c.vertices[1], c.vertices[2]));
		Edge e3 = std::make_pair(std::min(c.vertices[2], c.vertices[3]), std::max(c.vertices[2], c.vertices[3]));
		Edge e4 = std::make_pair(std::min(c.vertices[3], c.vertices[0]), std::max(c.vertices[3], c.vertices[0]));
		if (c.type == RIGID) {
			std::set<Edge> constraintEdges({e1, e2, e3, e4});
			std::pair<GridCell, std::set<Edge>> rigidConstraint = std::make_pair(c, constraintEdges);
			unlinkedConstraints.push_back(rigidConstraint);
		} else if (c.type == SHEAR) {
			std::set<Edge> constraintEdges1({e1, e3});
			std::set<Edge> constraintEdges2({e2, e4});
			std::pair<GridCell, std::set<Edge>> shearConstraint1 = std::make_pair(c, constraintEdges1);
			std::pair<GridCell, std::set<Edge>> shearConstraint2 = std::make_pair(c, constraintEdges2);
			unlinkedConstraints.push_back(shearConstraint1);
			unlinkedConstraints.push_back(shearConstraint2);
		}
	}

	//std::cout << unlinkedConstraints.size() << " unlinked constraints" << std::endl;

	// Reset constraint graph
	constraintGraph.clear();
	addConstraints(unlinkedConstraints);

	//std::cout << constraintGraph.size() << " components" << std::endl;
}

void GridModel::mergeComponents() {
	int currentDOFs = constraintGraph.size();
	int newDOFs = constraintGraph.size();
	srand (time(NULL)); // Initialize rng

	while (newDOFs >= currentDOFs)
	{

		// Select components to merge
	  int comp1 = -1;
		int comp2 = -1;

		if (constraintGraph.size() < 2) {
			std::cout << "Failed to merge: not enough components remain" << std::endl;
			break;
		}

		comp1 = rand() % constraintGraph.size();
		while (comp2 < 0 || comp2 == comp1) {
			comp2 = rand() % constraintGraph.size();
		}

		std::vector<GridCell> cellsComp1;
		std::vector<GridCell> cellsComp2;

		for (auto constraint : constraintGraph[comp1]) { // Get cells in first component
			if (std::find(cellsComp1.begin(), cellsComp1.end(), constraint.first) == cellsComp1.end()) {
				cellsComp1.push_back(constraint.first);
			}
		}

		for (auto constraint : constraintGraph[comp2]) { // Get cells in second component
			if (std::find(cellsComp2.begin(), cellsComp2.end(), constraint.first) == cellsComp2.end()) {
				cellsComp2.push_back(constraint.first);
			}
		}

		std::vector<GridCell> candidateCells;

		for (auto cell : cellsComp1) { // Collect cells common to both components
			if (std::find(cellsComp2.begin(), cellsComp2.end(), cell) != cellsComp2.end()) {
				candidateCells.push_back(cell);
			}
		}

		std::cout << "Trying to merge " << comp1 << " and " << comp2 << ". ";
		std::cout << "Candidates:";
		for (auto c : candidateCells) {
			std::cout << " (Cell: " << c.vertices[0] << ", " << c.vertices[1] << ", " << c.vertices[2] << ", " << c.vertices[3] << ")";
		}
		std::cout << std::endl;

		if (candidateCells.size() < 1) {
			std::cout << "Failed to merge: no cells common to both components." << std::endl;
			continue;
		}

		GridCell cellToMerge = candidateCells[rand() % candidateCells.size()];

		std::find(cells.begin(), cells.end(), cellToMerge)->type = RIGID;
		
		generateConstraintGraph();
		newDOFs = constraintGraph.size();
	}
	
}

std::vector<GridResult>
optimize(const GridModel& model, std::string pointDirectory)
{
	using namespace std;

	GridModel copy(model);

	MetaGrid grid(copy);
	grid.setEdges();
	grid.setEdgeRelations();

	std::vector<GridResult> ret;

	vector<int> sizes;
	for (auto& x : grid.constrained)
		sizes.push_back(x.second.size());

	int nframes = *min_element(sizes.begin(), sizes.end());

	double totError = .0;
	grid.setEdgeRelations();
	auto dofs = grid.degreesOfFreedom();

	SmartPtr<MyNLP> mynlp = new MyNLP(grid);
	mynlp->setStartConfiguration(grid.vertices);

	int cnt = 0;

	for (int i = 0; i < nframes; i += 1)
	{
		for (auto& x : grid.constrained)
		{
			mynlp->setConstraint(x.first, x.second[i]);
		}

		optimize(mynlp);
		totError += mynlp->objError;

		auto pts = grid.setDOFs(dofs, mynlp->solution);
		mynlp->setStartConfiguration(pts);

		GridResult resi;

		for (auto& p : pts) resi.points.push_back(p + grid.shift);

		if (!pointDirectory.empty())
		{
			ofstream file(pointDirectory + "/p" + to_string(i));
			for (auto& p : resi.points)
				file << p[0] << " " << p[1] << "\n";
			file.close();
		}

		ret.push_back(resi);
	}

	return ret;
}

Eigen::Vector2d*
buildAndOptimize(Eigen::Vector2d *points, int numPoints,
	int numCells, Eigen::Vector4i* cellsVertices, int* cellsTypes,
	int* anchors, int numAnchors,
	int* inputs, int numInputs, int* inputPathLenghts, Eigen::Vector2d* inputPaths,
	int* targets, int numTargets, int *targetPathLenghths, Eigen::Vector2d* targetPaths)
{
	GridModel model;

	for (size_t i = 0; i < numPoints; i++)
	{
		model.points.push_back(points[i]);
	}

	for (size_t i = 0; i < numCells; i++)
	{
		Eigen::Vector4i cellVertices = cellsVertices[i];
		GridCell cell(cellVertices[0], cellVertices[1], cellVertices[2], cellVertices[3], cellsTypes[i]);
		model.cells.push_back(cell);
	}

	for (size_t i = 0; i < numAnchors; i++)
	{
		model.anchors.push_back(anchors[i]);
	}

	int runner = 0;
	if (inputs != nullptr)
	{
		for (size_t i = 0; i < numInputs; i++)
		{
			int input = inputs[i];
			model.inputs.push_back(input);

			std::vector<Eigen::Vector2d> inputPath;

			for (size_t j = 0; j < inputPathLenghts[i]; j++)
			{
				Eigen::Vector2d point = inputPaths[runner];
				inputPath.push_back(point);
				runner++;
			}

			if (inputPath.size() > 0)
				model.inputPaths.push_back(inputPath);
		}
	}

	runner = 0;
	for (size_t i = 0; i < numTargets; i++)
	{
		int target = targets[i];
		model.targets.push_back(target);

		std::vector<Eigen::Vector2d> targetPath;

		for (size_t j = 0; j < targetPathLenghths[i]; j++)
		{
			Eigen::Vector2d point = targetPaths[runner];
			targetPath.push_back(point);
			runner++;
		}

		if (targetPath.size() > 0)
			model.targetPaths.push_back(targetPath);
	}
	
	std::vector<GridResult> results = optimize(model, "");

	std::vector<Eigen::Vector2d> *resultPoints = new std::vector<Eigen::Vector2d>();

	for (GridResult result : results)
	{
		for (Eigen::Vector2d point : result.points)
		{
			resultPoints->push_back(point);
		}
	}

	return &resultPoints->at(0);
};