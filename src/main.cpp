#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <igl/opengl/glfw/Viewer.h>
#include "external/Chipmunk2D/include/chipmunk/chipmunk.h"
#include "MMGrid.hpp"

using namespace std;
using namespace Eigen;

int main()
{
	int rows = 4;
	int cols = 2;

	vector<int> cells(rows * cols);

	const double bevel = 0.06;
	const double cellsize = 1;
	const cpFloat linkMass = .1;

	MMGrid myGrid(rows, cols, linkMass, bevel, cells);

	cpFloat timeStep = 1.0 / 60.0;
	// timeStep *= .5;

	igl::opengl::glfw::Viewer viewer;
	viewer.data().point_size = 20;
	viewer.data().set_face_based(true);
	viewer.core().orthographic = true;
	viewer.core().toggle(viewer.data().show_lines);
	viewer.core().is_animating = true;
	viewer.core().background_color.setOnes();

	int selected_cell = 0;
	viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer &v) -> bool
	{
		myGrid.render(&v, selected_cell);
		myGrid.update(timeStep);
		return false;
	};
	char editMode = 'r';
	viewer.callback_key_down = [&](igl::opengl::glfw::Viewer &, unsigned char key, int modifier) -> bool
	{
		// cout << "Key Pressed: " << key << "(" << (int)key << ")" << endl;
		if (key == 'R')
		{
			editMode = 'r';
		}
		else if (key == 'C')
		{
			editMode = 'c';
		}
		else if (key == '-')
		{
			if (editMode == 'r')
			{
				rows--;
			}
			else if (editMode == 'c')
			{
				cols--;
			}
			// cells.clear();
			cells = vector<int>(rows * cols);
			myGrid.setCells(rows, cols, cells);
		}
		else if (key == '=')
		{
			if (editMode == 'r')
			{
				rows++;
			}
			else if (editMode == 'c')
			{
				cols++;
			}
			// cells.clear();
			cells = vector<int>(rows * cols);
			myGrid.setCells(rows, cols, cells);
		}
		else if ((int)key == 6)
		{
			selected_cell++;
			selected_cell %= (rows * cols);
		}
		else if ((int)key == 7)
		{
			selected_cell--;
			selected_cell %= (rows * cols);
		}
		else if ((int)key == 8)
		{
			selected_cell += rows * cols - cols;
			selected_cell %= (rows * cols);
		}
		else if ((int)key == 9)
		{
			selected_cell += cols;
			selected_cell %= (rows * cols);
		}
		else if (key == 'M')
		{
			cells[selected_cell] = cells[selected_cell] == 1 ? 0 : 1;
			myGrid.setCells(rows, cols, cells);
		}
		else if (key == 'G')
		{
			cout << "apply leftward force on top link" << endl;
			myGrid.applyForce(1, selected_cell);
		}
		else if (key == 'K')
		{
			cout << "apply rightward force on top link" << endl;
			myGrid.applyForce(0, selected_cell);
		}
		else if (key == 'H')
		{
			cout << "apply upward force on left link" << endl;
			myGrid.applyForce(2, selected_cell);
		}
		else if (key == 'J')
		{
			cout << "apply downward force on right link" << endl;
			myGrid.applyForce(3, selected_cell);
		}
		else if (key == '9')
		{
			myGrid.setBevel(myGrid.getBevel() + .01);
		}
		else if (key == '7')
		{
			myGrid.setBevel(myGrid.getBevel() - .01);
		}
		else if (key == '3')
		{
			myGrid.setLinkMass(myGrid.getLinkMass() + .01);
		}
		else if (key == '1')
		{
			myGrid.setLinkMass(myGrid.getLinkMass() - .01);
		}
		return false;
	};
	// viewer.data().set_edges(points, edges, ec);
	viewer.launch();
}