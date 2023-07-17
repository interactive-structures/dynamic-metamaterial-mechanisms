#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include "misc/cpp/imgui_stdlib.h"
#include "chipmunk/chipmunk.h"
// #include "MMGrid.hpp"
// #include "PRand.hpp"
#include "SimulatedAnnealing.hpp"

using namespace std;
using namespace Eigen;

void printConstraints(ConstraintGraph cf)
{
	cout << "Constraints: [";
	for (auto rc : cf.getRowConstraints())
	{
		cout << rc;
	}
	for (auto cc : cf.getColConstraints())
	{
		cout << cc;
	}
	cout << "]" << endl;
}

int main()
{
	srand(time(NULL));
	int rows = 2;
	int cols = 2;
	int updatesPerRender = 5;

	vector<int> cells(rows * cols);

	const double cellsize = 1;

	MMGrid myGrid(rows, cols, cells);
	rows = myGrid.getRows();
	cols = myGrid.getCols();
	cells = myGrid.getCells();
	float stiffness = myGrid.getStiffness();
	float bevel = myGrid.getBevel();
	float damping = myGrid.getDamping();
	float linkMass = myGrid.getLinkMass();
	int shrink_factor = myGrid.getShrinkFactor();

	cpFloat timeStep = 5.0 / 60.0;

	igl::opengl::glfw::Viewer viewer;
	viewer.data().point_size = 20;
	viewer.data().set_face_based(true);
	viewer.core().orthographic = true;
	viewer.core().toggle(viewer.data().show_lines);
	viewer.core().is_animating = true;
	viewer.core().background_color.setOnes();

	igl::opengl::glfw::imgui::ImGuiPlugin plugin;
	viewer.plugins.push_back(&plugin);
	igl::opengl::glfw::imgui::ImGuiMenu menu;
	plugin.widgets.push_back(&menu);

	int selected_cell = 0;
	int selected_joint = 0;
	int select_mode = 0;
	bool edit_enabled = false;
	bool edit_enabled = false;
	char editMode = 'r';
	bool following_path = false;
	string config_file = "";

	menu.callback_draw_custom_window = [&]()
	{
		// Define next window position + size
		ImGui::SetNextWindowPos(ImVec2(180.f * menu.menu_scaling(), 10), ImGuiCond_FirstUseEver);
		ImGui::SetNextWindowSize(ImVec2(200, 500), ImGuiCond_FirstUseEver);
		ImGui::Begin(
			"Cell Config", nullptr,
			ImGuiWindowFlags_NoSavedSettings);

		ImGui::InputText("Config File", &config_file);
		if (ImGui::Button("Load Config File"))
		{
			myGrid.loadFromFile(config_file);
			rows = myGrid.getRows();
			cols = myGrid.getCols();
			cells = myGrid.getCells();
			float stiffness = myGrid.getStiffness();
			float bevel = myGrid.getBevel();
			float damping = myGrid.getDamping();
			float linkMass = myGrid.getLinkMass();
			int shrink_factor = myGrid.getShrinkFactor();
		}

		if (ImGui::Button("Path Following"))
		{
			following_path = !following_path;
		}

		if (ImGui::Button("Edit Enabled"))
		{
			edit_enabled = !edit_enabled;
			following_path = false;
			myGrid.setCells(rows, cols, cells);
		}

		if (select_mode == 0)
		{
			if (ImGui::Button("Cell Select Mode (Switch to Joint Select)"))
				select_mode = 1;
		}
		else
		{
			if (ImGui::Button("Joint Select Mode (Switch to Cell Select)"))
				select_mode = 0;
		}

		if (ImGui::Button("Toggle Selected Cell Rigid"))
		{
			cells[selected_cell] = cells[selected_cell] == 1 ? 0 : 1;
			myGrid.setCells(rows, cols, cells);
			ConstraintGraph cf(rows, cols, cells);
			printConstraints(cf);
		}

		if (ImGui::Button("Toggle Anchor Selected Joint"))
		{
			if (myGrid.isConstrained(selected_joint))
			{
				myGrid.removeJointController(selected_joint);
			}
			else
			{
				myGrid.addJointController(selected_joint);
			}
		}

		if (ImGui::Button("Shuffle cells w/ same CG"))
		{
			cout << "reconfiguring" << endl;
			ConstraintGraph cf(rows, cols, cells);
			cells = cf.makeCells();
			printConstraints(cf);
			myGrid.setCells(rows, cols, cells);
		}

		if (ImGui::Button("Maximal Constraint for CG"))
		{
			cout << "all cells" << endl;
			ConstraintGraph cf(rows, cols, cells);
			printConstraints(cf);
			cells = cf.allConstrainedCells();
			myGrid.setCells(rows, cols, cells);
		}

		if (ImGui::Button("Merge Components"))
		{
			cout << "merging" << endl;
			ConstraintGraph cf(rows, cols, cells);
			cf.mergeComponents();
			printConstraints(cf);
			cells = cf.makeCells();
			myGrid.setCells(rows, cols, cells);
		}

		if (ImGui::Button("Split Components"))
		{
			cout << "splitting" << endl;
			ConstraintGraph cf(rows, cols, cells);
			cf.splitComponents();
			printConstraints(cf);
			cells = cf.makeCells();
			myGrid.setCells(rows, cols, cells);
		}

		if (edit_enabled)
		{
			if (editMode == 'r')
			{
				if (ImGui::Button("Row Edit Mode (Switch to Column Edit)"))
					editMode = 'c';
			}
			else
			{
				if (ImGui::Button("Column Edit Mode (Switch to Row Edit)"))
					editMode = 'r';
			}
			if (ImGui::Button("increase"))
			{
				if (editMode == 'r')
				{
					rows++;
				}
				else if (editMode == 'c')
				{
					cols++;
				}
				cells = vector<int>(rows * cols);
				selected_cell = 0;
				selected_joint = 0;
				myGrid.setCells(rows, cols, cells);
			}
			if (ImGui::Button("decrease"))
			{
				if (editMode == 'r')
				{
					rows--;
				}
				else if (editMode == 'c')
				{
					cols--;
				}
				cells = vector<int>(rows * cols);
				selected_cell = 0;
				selected_joint = 0;
				myGrid.setCells(rows, cols, cells);
			}
			// Expose the same variable directly...
			ImGui::PushItemWidth(-80);
			if (ImGui::SliderFloat("Stiffness", &stiffness, 0.01, 10.0))
				myGrid.setStiffness(stiffness);
			ImGui::PopItemWidth();

			ImGui::PushItemWidth(-80);
			if (ImGui::SliderFloat("Damping", &damping, 0.01, 2.0))
				myGrid.setDamping(damping);
			ImGui::PopItemWidth();

			ImGui::PushItemWidth(-80);
			if (ImGui::SliderFloat("Linkmass", &linkMass, 0.01, 10.0))
				myGrid.setLinkMass(linkMass);
			ImGui::PopItemWidth();

			ImGui::PushItemWidth(-80);
			if (ImGui::SliderFloat("Bevel", &bevel, 0.01, 0.5))
				myGrid.setBevel(bevel);
			ImGui::PopItemWidth();

			ImGui::PushItemWidth(-80);
			if (ImGui::SliderInt("Shrink Factor", &shrink_factor, 1, 8))
				myGrid.setShrinkFactor(shrink_factor);
			ImGui::PopItemWidth();
		}

		ImGui::End();
	};

	viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer &v) -> bool
	{
		myGrid.render(&v, selected_cell, selected_joint);
		for (int i = 0; i < updatesPerRender; i++)
		{
			if (following_path)
			{
				myGrid.update_follow_path(timeStep / updatesPerRender, 1);
			}
			else
			{
				myGrid.update(timeStep / updatesPerRender);
			}
		}
		return false;
	};
	viewer.callback_key_down = [&](igl::opengl::glfw::Viewer &, unsigned char key, int modifier) -> bool
	{
		myGrid.setJointMaxForce(selected_joint, INFINITY);
		if ((int)key == 6)
		{
			if (select_mode == 0)
			{
				selected_cell++;
			}
			else
			{
				selected_joint++;
			}
			selected_cell %= (rows * cols);
			selected_joint %= (rows + 1) * (cols + 1);
		}
		else if ((int)key == 7)
		{
			if (select_mode == 0)
			{
				selected_cell += (rows * cols) - 1;
			}
			else
			{
				selected_joint += (rows + 1) * (cols + 1) - 1;
			}
			selected_cell %= (rows * cols);
			selected_joint %= (rows + 1) * (cols + 1);
		}
		else if ((int)key == 8)
		{
			if (select_mode == 0)
			{
				selected_cell += rows * cols - cols;
			}
			else
			{
				selected_joint += (rows + 1) * (cols + 1) - (cols + 1);
			}
			selected_cell %= (rows * cols);
			selected_joint %= (rows + 1) * (cols + 1);
		}
		else if ((int)key == 9)
		{
			if (select_mode == 0)
			{
				selected_cell += cols;
			}
			else
			{
				selected_joint += cols + 1;
			}
			selected_cell %= (rows * cols);
			selected_joint %= (rows + 1) * (cols + 1);
		}
		else if (key == '2')
		{
			cpVect d = cpv(0, -.1);
			myGrid.moveController(selected_joint, myGrid.getPos(selected_joint) + d);
		}
		else if (key == '4')
		{
			cpVect d = cpv(-.1, 0);
			myGrid.moveController(selected_joint, myGrid.getPos(selected_joint) + d);
		}
		else if (key == '6')
		{
			cpVect d = cpv(.1, 0);
			myGrid.moveController(selected_joint, myGrid.getPos(selected_joint) + d);
		}
		else if (key == '8')
		{
			cpVect d = cpv(0, .1);
			myGrid.moveController(selected_joint, myGrid.getPos(selected_joint) + d);
		}
		myGrid.setJointMaxForce(selected_joint, 500);
		return false;
	};

	viewer.launch();

	return 0;
}