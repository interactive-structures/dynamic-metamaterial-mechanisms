#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include "chipmunk/chipmunk.h"
#include "MMGrid.hpp"

using namespace std;
using namespace Eigen;

int main()
{
	int rows = 2;
	int cols = 2;
	int updatesPerRender = 5;

	vector<int> cells(rows * cols);


	const double cellsize = 1;


	MMGrid myGrid(rows, cols, cells);
	float stiffness = myGrid.getStiffness();
	float bevel = myGrid.getBevel();
	float damping = myGrid.getDamping();
	float linkMass = myGrid.getLinkMass();
	int shrink_factor = myGrid.getShrinkFactor();

	cpFloat timeStep = 1.0 / 60.0;
	// timeStep *= .5;

	igl::opengl::glfw::Viewer viewer;
	viewer.data().point_size = 20;
	viewer.data().set_face_based(true);
	viewer.core().orthographic = true;
	viewer.core().toggle(viewer.data().show_lines);
	viewer.core().is_animating = true;
	viewer.core().background_color.setOnes();

    // Eigen::Vector3d b(0,0,0);
    // std::pair a = generateCapsule(b, .1, 1, 10, 0);
    // std::pair c = generateCapsule(b, .1, 1, 10, M_PI / 2);
    // std::vector v = {c, a};
    // std::pair d = combineMeshes(v);
    // viewer.data().set_mesh(d.first, d.second);

	igl::opengl::glfw::imgui::ImGuiPlugin plugin;
	viewer.plugins.push_back(&plugin);
	igl::opengl::glfw::imgui::ImGuiMenu menu;
	plugin.widgets.push_back(&menu);

	menu.callback_draw_custom_window = [&]()
	{
	// Define next window position + size
	ImGui::SetNextWindowPos(ImVec2(180.f * menu.menu_scaling(), 10), ImGuiCond_FirstUseEver);
	ImGui::SetNextWindowSize(ImVec2(200, 160), ImGuiCond_FirstUseEver);
	ImGui::Begin(
		"Cell Config", nullptr,
		ImGuiWindowFlags_NoSavedSettings
	);

	// Expose the same variable directly ...
	ImGui::PushItemWidth(-80);
	if(ImGui::SliderFloat("Stiffness", &stiffness, 0.01, 10.0))
		myGrid.setStiffness(stiffness);
	ImGui::PopItemWidth();

	ImGui::PushItemWidth(-80);
	if(ImGui::SliderFloat("Damping", &damping, 0.01, 2.0))
		myGrid.setDamping(damping);
	ImGui::PopItemWidth();

	ImGui::PushItemWidth(-80);
	if(ImGui::SliderFloat("Linkmass", &linkMass, 0.01, 10.0))
		myGrid.setLinkMass(linkMass);
	ImGui::PopItemWidth();

	ImGui::PushItemWidth(-80);
	if(ImGui::SliderFloat("Bevel", &bevel, 0.01, 0.5))
		myGrid.setBevel(bevel);
	ImGui::PopItemWidth();

	ImGui::PushItemWidth(-80);
	if(ImGui::SliderInt("Shrink Factor", &shrink_factor, 1, 8))
		myGrid.setShrinkFactor(shrink_factor);
	ImGui::PopItemWidth();

	ImGui::End();
	};

	int selected_cell = 0;
	int selected_joint = 0;
	int select_mode = 0;
	viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer &v) -> bool
	{
		myGrid.render(&v, selected_cell, selected_joint);
		for(int i = 0; i < updatesPerRender; i++)
			myGrid.update(timeStep / updatesPerRender);
		return false;
	};
	char editMode = 'r';
	viewer.callback_key_down = [&](igl::opengl::glfw::Viewer &, unsigned char key, int modifier) -> bool
	{
		myGrid.setJointMaxForce(selected_joint, INFINITY);
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
			selected_cell = 0;
			selected_joint = 0;
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
			selected_cell = 0;
			selected_joint = 0;
			myGrid.setCells(rows, cols, cells);
		}
		else if ((int)key == 6)
		{
			if(select_mode == 0)
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
			if(select_mode == 0)
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
			if(select_mode == 0)
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
			if(select_mode == 0)
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
		else if (key == 'M')
		{
			cells[selected_cell] = cells[selected_cell] == 1 ? 0 : 1;
			myGrid.setCells(rows, cols, cells);
		}
		else if (key == 'J')
		{
			select_mode = select_mode == 0 ? 1 : 0;
		}
		else if (key == 'K')
		{
			if(myGrid.isConstrained(selected_joint)) {
				myGrid.removeJointController(selected_joint);
			}
			else {
				myGrid.addJointController(selected_joint);
			}
		}
		else if (key == '2') {
			cpVect d = cpv(0, -.1);
			myGrid.moveController(selected_joint, myGrid.getPos(selected_joint) + d);
		}
		else if (key == '4') {
			cpVect d = cpv(-.1, 0);
			myGrid.moveController(selected_joint, myGrid.getPos(selected_joint) + d);
		}
		else if (key == '6') {
			cpVect d = cpv(.1, 0);
			myGrid.moveController(selected_joint, myGrid.getPos(selected_joint) + d);
		}
		else if (key == '8') {
			cpVect d = cpv(0, .1);
			myGrid.moveController(selected_joint, myGrid.getPos(selected_joint) + d);
		}
		myGrid.setJointMaxForce(selected_joint, 500);
		// else if (key == 'G')
		// {
		// 	cout << "apply leftward force on top link" << endl;
		// 	myGrid.applyForce(1, selected_cell);
		// }
		// else if (key == 'K')
		// {
		// 	cout << "apply rightward force on top link" << endl;
		// 	myGrid.applyForce(0, selected_cell);
		// }
		// else if (key == 'H')
		// {
		// 	cout << "apply upward force on left link" << endl;
		// 	myGrid.applyForce(2, selected_cell);
		// }
		// else if (key == 'J')
		// {
		// 	cout << "apply downward force on right link" << endl;
		// 	myGrid.applyForce(3, selected_cell);
		// }
		return false;
	};
	// viewer.data().set_edges(points, edges, ec);
	viewer.launch();
    return 0;
}