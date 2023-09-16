#define _USE_MATH_DEFINES
#define _CRT_SECURE_NO_WARNINGS
#include <cmath>
#include <iostream>
#include <filesystem>
#include <fstream>

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include "misc/cpp/imgui_stdlib.h"
#include "common/UIModelData.hpp"
#include "common/SimulatedAnnealingSet.hpp"


#pragma once

void export_angles(std::string out_angle_file) {
	ofstream angleOutput(out_angle_file);
	if (angleOutput.good()) {
		int rows = UIModelData::modelDimensions[0];
		int cols = UIModelData::modelDimensions[1];
		vector<int> cells = UIModelData::cells;
		vector<int> activeCellIndices = {};
		for (int i = 0; i < cells.size(); i++) {
			if (cells[i] == 2) activeCellIndices.push_back(i);
		}
		UIModelData::modelGrid().setCells(rows, cols, cells);
		angleOutput << "Indices: ";
		for (int index : activeCellIndices)
		{
			angleOutput << index << ",";
		}
		angleOutput << std::endl;
		vector<vector<double>> angles = UIModelData::modelGrid().getAnglesFor(activeCellIndices);
		for (int i = 0; i < activeCellIndices.size(); i++)
		{
			angleOutput << "For cell " << activeCellIndices[i] << "(" << activeCellIndices[i] / cols << " " << activeCellIndices[i] % cols << "): " << std::endl;
			angleOutput << "Angles:" << std::endl;
			for (double d : angles[i])
			{
				angleOutput << d * 180 / M_PI << std::endl;
			}
			angleOutput << std::endl;
		}
		angleOutput.close();
	}
	else {
		std::cout << "FILE NOT FOUND!" << std::endl;
	}
}

void main_draw_debug()
{
	UIModelData::gridSet.reserve(10);
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
	int rc[] = { 1,1 };

	menu.callback_draw_viewer_window = [&]()
		{
			ImGuiIO io = ImGui::GetIO();
			ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);
			ImGui::SetNextWindowSize(ImVec2(0.0f, io.DisplaySize.y), ImGuiCond_FirstUseEver);
			ImGui::Begin(
				"editor", nullptr,
				ImGuiWindowFlags_NoSavedSettings
				| ImGuiWindowFlags_AlwaysAutoResize
			);
			ImGui::PushItemWidth(80 * menu.menu_scaling());
			// model
			if (ImGui::CollapsingHeader("MODEL", ImGuiTreeNodeFlags_DefaultOpen))
			{
				float w = ImGui::GetContentRegionAvail().x;
				float p = ImGui::GetStyle().FramePadding.x;
				if (ImGui::InputInt2("rows x cols##MODEL", rc)) {
					//std::cout << rc[0] << rc[1] << std::endl;
					UIModelData::dimChanged = true;
				};
				ImGui::Text("change selected cell type");
				if (ImGui::Button("shear##MODEL", ImVec2((w - 2 * p) / 3.f, 0)))
				{
					UIModelData::cells[UIModelData::selectedCell] = 0;
					UIModelData::cellsEdited = true;
				}
				ImGui::SameLine(0, p);
				if (ImGui::Button("rigid##MODEL", ImVec2((w - 2 * p) / 3.f, 0)))
				{
					UIModelData::cells[UIModelData::selectedCell] = 1;
					UIModelData::cellsEdited = true;
				}
				ImGui::SameLine(0, p);
				if (ImGui::Button("active##MODEL", ImVec2((w - 2 * p) / 3.f, 0)))
				{
					UIModelData::cells[UIModelData::selectedCell] = 2;
					UIModelData::cellsEdited = true;
				}
				if (ImGui::Button("reset all", ImVec2(w, 0))) {
					UIModelData::dimChanged = true;
				};
			}
			if (ImGui::CollapsingHeader("IMPORT", ImGuiTreeNodeFlags_DefaultOpen))
			{
				ImGui::Text("Imported Paths:");
				if (UIModelData::paths.size() > 0)
					for (auto& pair : UIModelData::paths) {
						string title = pair.first.substr(pair.first.length() - 8);
						ImGui::BulletText("...%s", title.c_str());
					}
				else {
					ImGui::BulletText("none...");
				}
				float w = ImGui::GetContentRegionAvail().x;
				float p = ImGui::GetStyle().FramePadding.x;
				if (ImGui::Button("import model##IMPORT", ImVec2(w, 0))) {
					std::string modelPath = igl::file_dialog_open();
					std::cout << modelPath << std::endl;
					//UIModelData::modelGrid() = MMGrid(1, 1, { 0 });
					UIModelData::modelGrid().loadFromFile(modelPath);
					UIModelData::cells = UIModelData::modelGrid().getCells();
					UIModelData::modelDimensions = { UIModelData::modelGrid().getRows(), UIModelData::modelGrid().getCols() };
					rc[0] = UIModelData::modelDimensions[0];
					rc[1] = UIModelData::modelDimensions[1];
				}
				if (ImGui::Button("import path##IMPORT", ImVec2(w, 0))) {
					std::string modelPath = igl::file_dialog_open();
					auto p = std::make_pair(modelPath, UIModelData::modelGrid().readPath(modelPath));
					if(p.second.size() > 0)
						UIModelData::paths.insert(p);

				};
				if (ImGui::Button("edit paths##IMPORT", ImVec2(w, 0))) {
					UIModelData::joint_path_editor_visible = !UIModelData::joint_path_editor_visible;
				};
				if (ImGui::Button("anchor joint", ImVec2(w, 0))) {
					UIModelData::modelGrid().anchor(UIModelData::selectedJoint);
				}
				if (ImGui::Button("unanchor joint", ImVec2(w, 0))) {
					UIModelData::modelGrid().unanchor(UIModelData::selectedJoint);
				}
				if (ImGui::Button("<##IMPORT", ImVec2((w - 2 * p) / 5.f, 0)))
				{
					UIModelData::gridIndex += UIModelData::gridSet.size() - 1;
					UIModelData::gridIndex %= UIModelData::gridSet.size();
					UIModelData::cellsEdited = true;
				}
				ImGui::SameLine(0, p);
				ImGui::Text("Set %d", UIModelData::gridIndex, ImVec2(3 * (w - 2 * p) / 5.f, 0));
				ImGui::SameLine(0, p);
				if (ImGui::Button(">##IMPORT", ImVec2((w - 2 * p) / 5.f, 0)))
				{
					UIModelData::gridIndex++;
					UIModelData::gridIndex %= UIModelData::gridSet.size();
					UIModelData::cellsEdited = true;
				}
				if (ImGui::Button("add set##IMPORT", ImVec2(w, 0))) {
					//std::cout << "Creating new Grid and add it to vector" << std::endl;
					UIModelData::gridSet.push_back(MMGrid(UIModelData::gridSet[UIModelData::gridIndex]));
					//std::cout << "Getting size" << std::endl;
					UIModelData::gridIndex = UIModelData::gridSet.size() - 1;
					//std::cout << "Should be getting reference or something" << std::endl;
					UIModelData::cellsEdited = true;
				};
			}
			if (ImGui::CollapsingHeader("SIMULATE", ImGuiTreeNodeFlags_DefaultOpen))
			{
				float w = ImGui::GetContentRegionAvail().x;
				float p = ImGui::GetStyle().FramePadding.x;
				if (ImGui::Button("<<##SIMULATE", ImVec2((w - 2 * p) / 3.f, 0)))
				{
					UIModelData::modelGrid().prevPoint();
				}
				ImGui::SameLine(0, p);
				if (UIModelData::playing) {
					if (ImGui::Button("||##SIMULATE", ImVec2((w - 2 * p) / 3.f, 0)))
					{
						UIModelData::playing = false;
					}
				}
				else {
					if (ImGui::Button("|>##SIMULATE", ImVec2((w - 2 * p) / 3.f, 0)))
					{
						UIModelData::playing = true;
					}
				}
				ImGui::SameLine(0, p);
				if (ImGui::Button(">>##SIMULATE", ImVec2((w - 2 * p) / 3.f, 0)))
				{
					UIModelData::modelGrid().nextPoint();
				}
				if (ImGui::Button("reset simulation", ImVec2(w, 0))) {
					UIModelData::cellsEdited = true;
				};
				if (ImGui::Button("playback options")) {
					UIModelData::playback_options_visible = !UIModelData::playback_options_visible;
				};
			}
			if (ImGui::CollapsingHeader("OPTIMIZE", ImGuiTreeNodeFlags_DefaultOpen))
			{
				float w = ImGui::GetContentRegionAvail().x;
				ImGui::InputInt("# iterations", &UIModelData::annealingSteps);
				if (ImGui::Button("optimize for paths", ImVec2(w, 0))) {
					UIModelData::allCalculatedPaths.clear();
					UIModelData::allCalculatedPaths.resize(UIModelData::gridSet.size());
					SimulatedAnnealingSet sa(UIModelData::gridSet, UIModelData::pathWeight, UIModelData::dofWeight);
					MMGrid out = sa.simulate(UIModelData::annealingSteps);
					int index = 0;
					for (MMGrid& grid : UIModelData::gridSet) {
						grid.setCalculatedPaths(UIModelData::allCalculatedPaths[index]);
						index++;
					}
					UIModelData::cells = out.getCells();
					UIModelData::cellsEdited = true;
				}
				if (ImGui::Button("edit optimization weights", ImVec2(w, 0))) {
					UIModelData::opt_wseights_visible = !UIModelData::opt_wseights_visible;
				};
			}
			if (ImGui::CollapsingHeader("EXPORT"))
			{
				float w = ImGui::GetContentRegionAvail().x;
				if (ImGui::Button("export model", ImVec2(w, 0))) {
					std::string modelPath = igl::file_dialog_save();
					UIModelData::modelGrid().writeModel(modelPath);
				};
				if (ImGui::Button("export model and paths", ImVec2(w, 0))) {
					std::string modelPath = igl::file_dialog_save();
					UIModelData::modelGrid().writeConfig(modelPath);
				};
				if (ImGui::Button("export angles", ImVec2(w, 0))) {
					std::string out_angle_file = igl::file_dialog_save();
					export_angles(out_angle_file);
				};
			}
			if (ImGui::CollapsingHeader("ADVANCED"))
			{
				float w = ImGui::GetContentRegionAvail().x;
				if (ImGui::Button("edit simulation parameters", ImVec2(w, 0))) {
					UIModelData::sim_params_visible = !UIModelData::sim_params_visible;
				};
				if (ImGui::Button("view simulation info", ImVec2(w, 0))) {
					UIModelData::sim_info_visible = !UIModelData::sim_info_visible;
				};
			}
			ImGui::PopItemWidth();
			ImGui::End();
		};

	menu.callback_draw_custom_window = [&]()
		{
			if (UIModelData::joint_path_editor_visible) {
				ImGui::SetNextWindowPos(ImVec2(180.f * menu.menu_scaling(), 10), ImGuiCond_FirstUseEver);
				ImGui::SetNextWindowSize(ImVec2(200, 500), ImGuiCond_FirstUseEver);
				std::ostringstream titlestream;
				titlestream << "Joint " << UIModelData::selectedJoint << " Paths";
				ImGui::Begin(
					titlestream.str().c_str(), &UIModelData::joint_path_editor_visible,
					ImGuiWindowFlags_NoSavedSettings);
				auto pathForJoint = UIModelData::modelGrid().getPathFor(UIModelData::selectedJoint);

				float w = ImGui::GetContentRegionAvail().x;
				float p = ImGui::GetStyle().FramePadding.x;
				if (ImGui::BeginCombo("Imported Paths", UIModelData::pathSelection.c_str())) {
					for (auto kv : UIModelData::paths) {
						bool is_selected = kv.first == UIModelData::pathSelection;
						if (ImGui::Selectable(kv.first.c_str(), is_selected)) {
							UIModelData::pathSelection = kv.first;
						}
						if (is_selected)
							ImGui::SetItemDefaultFocus();
					}
					bool is_selected = "" == UIModelData::pathSelection;
					if (ImGui::Selectable("None", is_selected)) {
						UIModelData::pathSelection = "";
					}
					if (is_selected)
						ImGui::SetItemDefaultFocus();
					ImGui::EndCombo();
				}

				if (ImGui::Button("Set Path to Selected")) {
					UIModelData::modelGrid().setPath(UIModelData::paths[UIModelData::pathSelection],
						UIModelData::selectedJoint);
					UIModelData::pathScale = 1.0;
				}

				if (pathForJoint.size() == 0) {
					ImGui::Text("No existing paths");
				}
				else {
					ImGui::Text("Existing path found!");
					ImGui::InputFloat("Scale", &UIModelData::pathScale);
					ImGui::SameLine();
					if (ImGui::Button("Set Scale")) {
						UIModelData::modelGrid().scalePath(UIModelData::pathScale, UIModelData::selectedJoint);
						UIModelData::pathScale = 1.0;
					}
					if (ImGui::Button("Remove Existing Path")) {
						UIModelData::modelGrid().removePath(UIModelData::selectedJoint);
					}
				}
				ImGui::End();
			};
			if (UIModelData::opt_wseights_visible) {
				ImGui::SetNextWindowPos(ImVec2(180.f * menu.menu_scaling(), 10), ImGuiCond_FirstUseEver);
				ImGui::SetNextWindowSize(ImVec2(200, 500), ImGuiCond_FirstUseEver);
				ImGui::Begin(
					"Optimization Weights", &UIModelData::opt_wseights_visible,
					ImGuiWindowFlags_NoSavedSettings);
				ImGui::SliderFloat("Path Weight", &UIModelData::pathWeight, 0.0, 10.0);
				ImGui::SliderFloat("DOF Weight", &UIModelData::dofWeight, 0.0, 10.0);
				ImGui::End();
			};
			if (UIModelData::playback_options_visible) {
				ImGui::SetNextWindowPos(ImVec2(180.f * menu.menu_scaling(), 10), ImGuiCond_FirstUseEver);
				ImGui::SetNextWindowSize(ImVec2(200, 500), ImGuiCond_FirstUseEver);
				ImGui::Begin(
					"Playback Options", &UIModelData::playback_options_visible,
					ImGuiWindowFlags_NoSavedSettings);
				ImGui::InputFloat("Simulation Timestep", &UIModelData::simTimestep);
				ImGui::InputFloat("Playback Points Per Second", &UIModelData::playbackPointsPerSecond);
				ImGui::End();
			};
			if (UIModelData::sim_info_visible) {
				ImGui::SetNextWindowPos(ImVec2(180.f * menu.menu_scaling(), 10), ImGuiCond_FirstUseEver);
				ImGui::SetNextWindowSize(ImVec2(200, 500), ImGuiCond_FirstUseEver);
				ImGui::Begin(
					"Simulation Info", &UIModelData::sim_info_visible,
					ImGuiWindowFlags_NoSavedSettings);
				ImGui::Text("For Selected Joint");

				ImGui::Text("For Selected Cell");
				float angle = UIModelData::modelGrid().getCurrentAngle(UIModelData::selectedCell);
				ImGui::Text("  Current Angle: %f", angle / M_PI_2 * 90.f);
				ImGui::End();
			};
			if (UIModelData::sim_params_visible) {
				ImGui::SetNextWindowPos(ImVec2(180.f * menu.menu_scaling(), 10), ImGuiCond_FirstUseEver);
				ImGui::SetNextWindowSize(ImVec2(200, 500), ImGuiCond_FirstUseEver);
				ImGui::Begin(
					"Simulation Parameters", &UIModelData::sim_params_visible,
					ImGuiWindowFlags_NoSavedSettings);
				// Expose the same variable directly...

				float stiffness = UIModelData::modelGrid().getStiffness(), damping = UIModelData::modelGrid().getDamping(), linkMass = UIModelData::modelGrid().getLinkMass(),
					bevel = UIModelData::modelGrid().getBevel();
				int shrink_factor = UIModelData::modelGrid().getShrinkFactor();

				ImGui::PushItemWidth(-80);
				if (ImGui::SliderFloat("Stiffness", &stiffness, 0.01, 10.0))
					UIModelData::modelGrid().setStiffness(stiffness);
				ImGui::PopItemWidth();

				ImGui::PushItemWidth(-80);
				if (ImGui::SliderFloat("Damping", &damping, 0.01, 2.0))
					UIModelData::modelGrid().setDamping(damping);
				ImGui::PopItemWidth();

				ImGui::PushItemWidth(-80);
				if (ImGui::SliderFloat("Linkmass", &linkMass, 0.01, 10.0))
					UIModelData::modelGrid().setLinkMass(linkMass);
				ImGui::PopItemWidth();

				ImGui::PushItemWidth(-80);
				if (ImGui::SliderFloat("Bevel", &bevel, 0.01, 0.5))
					UIModelData::modelGrid().setBevel(bevel);
				ImGui::PopItemWidth();

				ImGui::PushItemWidth(-80);
				if (ImGui::SliderInt("Shrink Factor", &shrink_factor, 1, 8))
					UIModelData::modelGrid().setShrinkFactor(shrink_factor);
				ImGui::PopItemWidth();
				ImGui::End();
			};
		};

	viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer& v) -> bool
		{
			if (UIModelData::dimChanged) {
				int rows = rc[0];
				int cols = rc[1];
				UIModelData::modelDimensions = { rows, cols };
				UIModelData::cells.clear();
				UIModelData::cells.resize(rows * cols);
				for (MMGrid& grid : UIModelData::gridSet) {
					grid.setCells(rows, cols, UIModelData::cells);
				}
				UIModelData::dimChanged = false;
			}
			if (UIModelData::cellsEdited) {
				int rows = UIModelData::modelDimensions[0];
				int cols = UIModelData::modelDimensions[1];
				for (MMGrid& grid : UIModelData::gridSet) {
					grid.setCells(rows, cols, UIModelData::cells);
				}
				UIModelData::cellsEdited = false;
			}
			if (UIModelData::sim_running) {
				if (UIModelData::playing) {
					UIModelData::modelGrid().update_follow_path(UIModelData::simTimestep, UIModelData::playbackPointsPerSecond);
				}
				else {
					UIModelData::modelGrid().update(UIModelData::simTimestep);
				}
			}
			UIModelData::modelGrid().render(&v, UIModelData::selectedCell, UIModelData::selectedJoint);
			return false;
		};
	viewer.callback_key_down = [&](igl::opengl::glfw::Viewer&, unsigned char key, int modifier) -> bool
		{
			if (modifier == 1) {
				//shift key
				if (key == 7) {
					UIModelData::selectedJoint--;
				}
				if (key == 6) {
					UIModelData::selectedJoint++;
				}
				if (key == 8) {
					UIModelData::selectedJoint -= (UIModelData::modelDimensions[1] + 1);
				}
				if (key == 9) {
					UIModelData::selectedJoint += (UIModelData::modelDimensions[1] + 1);
				}
				UIModelData::selectedJoint += (UIModelData::modelDimensions[0] + 1) * (UIModelData::modelDimensions[1] + 1);
				UIModelData::selectedJoint %= (UIModelData::modelDimensions[0] + 1) * (UIModelData::modelDimensions[1] + 1);
			}
			else {
				if (key == 7) {
					UIModelData::selectedCell--;
				}
				if (key == 6) {
					UIModelData::selectedCell++;
				}
				if (key == 8) {
					UIModelData::selectedCell -= UIModelData::modelDimensions[1];
				}
				if (key == 9) {
					UIModelData::selectedCell += UIModelData::modelDimensions[1];
				}
				UIModelData::selectedCell += UIModelData::modelDimensions[0] * UIModelData::modelDimensions[1];
				UIModelData::selectedCell %= UIModelData::modelDimensions[0] * UIModelData::modelDimensions[1];
			}
			return false;
		};

	viewer.launch();
}

int main()
{
	main_draw_debug();
	return 0;
}