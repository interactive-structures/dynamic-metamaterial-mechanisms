#ifndef Communicator_h
#define Communicator_h

#include <vector>

#ifdef _WIN32
#define EIGEN_DONT_ALIGN_STATICALLY
#endif

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <array>
#include "GridModel.h"
#include <cliext/vector>

class Communicator
{
public:

	std::vector<Eigen::Vector2d> points;
	std::vector<GridCell> cells;
	std::vector<int> anchors;
	std::vector<int>inputs;
	std::vector<std::vector<Eigen::Vector2d>>inputPaths;
	std::vector<int>targets;
	std::vector<std::vector<Eigen::Vector2d>>targetPaths;

	Communicator()
	{
	};

	void addPoint(double x, double y)
	{
		points.push_back(Eigen::Vector2d(x, y));
	}

	void addCell(const int a, const int b, const int c, const int d, const int t)
	{
		GridCell cell(a, b, c, d, t);
		cells.push_back(cell);
	}

	void addAnchor(int anchor)
	{
		anchors.push_back(anchor);
	}

	void addInput(int input)
	{
		inputs.push_back(input);
		inputPaths.push_back(std::vector<Eigen::Vector2d>());
	}

	void addInputPathPoint(int index, double x, double y)
	{
		inputPaths[index].push_back(Eigen::Vector2d(x, y));
	}

	void addTarget(int target)
	{
		targets.push_back(target);
		targetPaths.push_back(std::vector<Eigen::Vector2d>());
	}

	void addTargetPathPoint(int index, double x, double y)
	{
		targetPaths[index].push_back(Eigen::Vector2d(x, y));
	}

	Eigen::Vector2d* optimizeNative()
	{
		std::vector<int> cellsTypes;
		std::vector<Eigen::Vector4i> cellsVertices;

		for (size_t i = 0; i < cells.size(); i++)
		{
			cellsTypes.push_back(cells[i].type);
			cellsVertices.push_back(cells[i].vertices);
		}

		std::vector<int> targetPathLengths;
		std::vector<Eigen::Vector2d> convertedTargetPaths;

		for (std::vector<Eigen::Vector2d> path : targetPaths)
		{
			targetPathLengths.push_back(path.size());
			for (Eigen::Vector2d point : path)
			{
				convertedTargetPaths.push_back(point);
			}
		}

		Eigen::Vector2d* results;

		if (inputPaths.size() > 0)
		{
			std::vector<int> inputPathLengths;

			std::vector<Eigen::Vector2d> convertedInputPaths;
			for (std::vector<Eigen::Vector2d> path : inputPaths)
			{
				inputPathLengths.push_back(path.size());
				for (Eigen::Vector2d point : path)
				{
					convertedInputPaths.push_back(point);
				}
			}
			results = buildAndOptimize(&points[0], points.size(),
				cells.size(), &cellsVertices[0], &cellsTypes[0], 
				&anchors[0], anchors.size(), 
				&inputs[0], inputs.size(), &inputPathLengths[0], &convertedInputPaths[0],  
				&targets[0], targets.size(), &targetPathLengths[0], &convertedTargetPaths[0]);
		}
		else
		{
			results = buildAndOptimize(&points[0], points.size(),
				cells.size(), &cellsVertices[0], &cellsTypes[0], 
				&anchors[0], anchors.size(),
				nullptr, 0, nullptr, nullptr,
				&targets[0], targets.size(), &targetPathLengths[0], &convertedTargetPaths[0]);
		}

		return results;
	}
};

#endif /* GridModel_h */
