#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include "chipmunk/chipmunk.h"
// #include "MMGrid.hpp"
// #include "PRand.hpp"
// #include "common/SimulationSpace.hpp"
// #include "common/SimulatedAnnealing.hpp"
#include "common/Renderer.hpp"
#include "common/Mechanism.hpp"

using namespace std;
using namespace Eigen;

void main_draw_debug()
{

    cpFloat timeStep = 1.0 / 60.0;

    igl::opengl::glfw::Viewer viewer;
    viewer.data().point_size = 20;
    viewer.data().set_face_based(true);
    viewer.core().orthographic = true;
    viewer.core().toggle(viewer.data().show_lines);
    viewer.core().is_animating = true;
    viewer.core().background_color.setOnes();

    bool animating = false;
    bool controller_enabled = false;

    igl::opengl::glfw::imgui::ImGuiPlugin plugin;
    viewer.plugins.push_back(&plugin);
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    plugin.widgets.push_back(&menu);
    Renderer r(8);
    Mechanism mm = Grid(1,2);
    SimulatedMechanism sm = mm.makeSimulation(.4/120, 1, 1, .1, 1, 1, .3);
    sm.space.setGravity({0,-1});
    for(auto c : sm.cells) {
        r.addCell("cell", c.corners, 1, .15);
    }
    // SimulationSpace space(0.4/120);
    // SimulationBody link1 = space.addSegmentBody({0,0}, {0,1}, 1, .05);
    // SimulationBody link2 = space.addSegmentBody({0,0}, {1,0}, 1, .05);
    // SimulationBody controller = space.addKinematicBody({0,0.5});
    // SimulationConstraint controlConstraint = space.pivotConstrain(link1, controller, {0, 0.5});
    // SimulationConstraint spring = space.rotarySpringConstrain(link1, link2, 0, 1, 1);
    // SimulationConstraint pivot = space.pivotConstrain(link1, link2, {0,0});
    // SimulationBody ground = space.addStaticSegmentBody({-10,-.5}, {10,-.5}, .05);
    // space.setGravity({0,-1});

    menu.callback_draw_custom_window = [&]()
    {
        // Define next window position + size
        ImGui::SetNextWindowPos(ImVec2(180.f * menu.menu_scaling(), 10), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(200, 500), ImGuiCond_FirstUseEver);
        ImGui::Begin(
            "Cell Config", nullptr,
            ImGuiWindowFlags_NoSavedSettings);

        ImGui::End();
    };

    clock_t anim_start, anim_end;

    viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer &v) -> bool
    {
        // MatrixX2i edges(3,2);
        // MatrixX3d points = MatrixXd::Zero(8,3);
        // points.row(0) = Vector3d(link1.getPos()[0], link1.getPos()[1], 0);
        // points.row(1) = Vector3d(link2.getPos()[0], link2.getPos()[1], 0);
        // points.row(2) = Vector3d(link1.getGlobalSegmentPosA()[0], link1.getGlobalSegmentPosA()[1], 0);
        // points.row(3) = Vector3d(link1.getGlobalSegmentPosB()[0], link1.getGlobalSegmentPosB()[1], 0);
        // points.row(4) = Vector3d(link2.getGlobalSegmentPosA()[0], link2.getGlobalSegmentPosA()[1], 0);
        // points.row(5) = Vector3d(link2.getGlobalSegmentPosB()[0], link2.getGlobalSegmentPosB()[1], 0);
        // points.row(6) = Vector3d(ground.getGlobalSegmentPosA()[0], ground.getGlobalSegmentPosA()[1], 0);
        // points.row(7) = Vector3d(ground.getGlobalSegmentPosB()[0], ground.getGlobalSegmentPosB()[1], 0);
        // edges << 2,3,
        //         4,5,
        //         6,7;
        // if(!controller_enabled)
        // {
        //     controlConstraint.setMaxForce(0.0);
        //     controller.setPosition(link1.getPos());
        // } else {
        //     controlConstraint.setMaxForce(INFINITY);
        // }
        // if(animating) {
        //     anim_start = clock();
        //     for(size_t i = 0; i < 5; i++)
        //     space.step();
        //     anim_end = clock();
        //     std::cout << "Update took: (double) " << double(anim_end - anim_start) / double(CLOCKS_PER_SEC) << " seconds..." << std::endl;
        // }
        // v.data().set_points(points, MatrixXd::Zero(8,3));
        // v.data().set_edges(points, edges, MatrixXd::Zero(3,3));
        r.clear();
        for(auto c : sm.cells) {
            r.addCell("cell", c.corners, 1, .15);
        }
        // sm.step();
        r.renderTo(v);
        return false;
    };
    viewer.callback_key_down = [&](igl::opengl::glfw::Viewer &, unsigned char key, int modifier) -> bool
    {
        // if(key == ' ') {
        //     animating = !animating;
        //     std::cout << "Animating is " << animating << std::endl;
        // }
        // else if (key == 'C') {
        //     controller_enabled = !controller_enabled;
        //     std::cout << "Controller_Enabled is " << controller_enabled << std::endl;
        // }
        // else if ((int)key == 6)
        // {
        //     controller.changePosition({0.1, 0});
        // }
        // else if ((int)key == 7)
        // {
        //     controller.changePosition({-0.1, 0});
        // }
        // else if ((int)key == 8)
        // {
        //     controller.changePosition({0, -0.1});
        // }
        // else if ((int)key == 9)
        // {
        //     controller.changePosition({0,  0.1});
        // }
        //
        return false;
    };

    viewer.launch();
}
int main()
{
    main_draw_debug();
    return 0;
}