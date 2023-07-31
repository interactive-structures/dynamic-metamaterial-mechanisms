#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include "chipmunk/chipmunk.h"
// #include "MMGrid.hpp"
// #include "PRand.hpp"
#include "common/SimulatedAnnealing.hpp"

using namespace std;
using namespace Eigen;

cpVect getWorldPosA(cpBody* body, cpShape *segment) {
    cpVect rotated = cpBodyGetRotation(body);
    cpVect translatedPosA = cpvrotate(cpSegmentShapeGetA(segment), rotated);
    return cpBodyGetPosition(body) + translatedPosA;
}

cpVect getWorldPosB(cpBody* body, cpShape *segment) {
    cpVect rotated = cpBodyGetRotation(body);
    cpVect translatedPosB = cpvrotate(cpSegmentShapeGetB(segment), rotated);
    return cpBodyGetPosition(body) + translatedPosB;
}

void main_draw_debug(){

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
    cpSpace *space = cpSpaceNew();
    cpBody *link1 = cpSpaceAddBody(space, cpBodyNew(1, cpMomentForSegment(1, cpvzero, cpv(1,0), .05)));
    cpShape *linkshape1 = cpSpaceAddShape(space, cpSegmentShapeNew(link1, cpv(0,-.35), cpv(0,.35), .05));
    cpBody *link2 = cpSpaceAddBody(space, cpBodyNew(1, cpMomentForSegment(1, cpvzero, cpv(1,0), .05)));
    cpShape *linkshape2 = cpSpaceAddShape(space, cpSegmentShapeNew(link2, cpv(-.35, 0), cpv(.35, 0), .05));
    cpBody *controller = cpSpaceAddBody(space, cpBodyNewKinematic());
    cpConstraint *controlConstraint = cpSpaceAddConstraint(space, cpPivotJointNew2(link1, controller, cpvzero, cpvzero));
    cpBodySetPosition(link1, cpv(0,0.5));
    cpBodySetPosition(link2, cpv(0.5,0));
    cpBodySetPosition(controller, cpv(0,0.5));
    cpConstraint *spring = cpSpaceAddConstraint(space, cpDampedRotarySpringNew(link1, link2, 0, 1, 1));
    cpConstraint *pivot = cpSpaceAddConstraint(space, cpPivotJointNew(link1, link2, cpvzero));
    cpShape *ground = cpSpaceAddShape(space, cpSegmentShapeNew(cpSpaceGetStaticBody(space), cpv(-10,-.5), cpv(10, -.5), .05));
    cpSpaceSetGravity(space, cpv(0,-1));

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
		v.data().clear();
        MatrixX2i edges(3,2);
        MatrixX3d points = MatrixXd::Zero(8,3);
        points.row(0) = Vector3d(cpBodyGetPosition(link1).x, cpBodyGetPosition(link1).y, 0);
        points.row(1) = Vector3d(cpBodyGetPosition(link2).x, cpBodyGetPosition(link2).y, 0);
        points.row(2) = Vector3d(getWorldPosA(link1, linkshape1).x, getWorldPosA(link1, linkshape1).y, 0);
        points.row(3) = Vector3d(getWorldPosB(link1, linkshape1).x, getWorldPosB(link1, linkshape1).y, 0);
        points.row(4) = Vector3d(getWorldPosA(link2, linkshape2).x, getWorldPosA(link2, linkshape2).y, 0);
        points.row(5) = Vector3d(getWorldPosB(link2, linkshape2).x, getWorldPosB(link2, linkshape2).y, 0);
        points.row(6) = Vector3d(cpSegmentShapeGetA(ground).x, cpSegmentShapeGetA(ground).y, 0);
        points.row(7) = Vector3d(cpSegmentShapeGetB(ground).x, cpSegmentShapeGetB(ground).y, 0);
        edges << 2,3,
                4,5,
                6,7;
        if(!controller_enabled)
        {
            cpConstraintSetMaxForce(controlConstraint, 0.0f);
            cpBodySetPosition(controller, cpBodyGetPosition(link1));
        } else {
            cpConstraintSetMaxForce(controlConstraint, INFINITY);
        }
        if(animating) {
            anim_start = clock();
            for(size_t i = 0; i < 5; i++)
            cpSpaceStep(space, 0.4/120.0);
            anim_end = clock();
            std::cout << "Update took: (double) " << double(anim_end - anim_start) / double(CLOCKS_PER_SEC) << " seconds..." << std::endl;
        }
        v.data().set_points(points, MatrixXd::Zero(8,3));
        v.data().set_edges(points, edges, MatrixXd::Zero(3,3));
		return false;
	};
	viewer.callback_key_down = [&](igl::opengl::glfw::Viewer &, unsigned char key, int modifier) -> bool
	{
        if(key == ' ') {
            animating = !animating;
            std::cout << "Animating is " << animating << std::endl;
        }
        else if (key == 'C') {
            controller_enabled = !controller_enabled;
        }
        else if ((int)key == 6)
		{
			cpVect d = cpv(0.1, 0);
            cpBodySetPosition(controller, cpBodyGetPosition(controller) + d);
		}
		else if ((int)key == 7)
		{
			cpVect d = cpv(-0.1, 0);
            cpBodySetPosition(controller, cpBodyGetPosition(controller) + d);
		}
		else if ((int)key == 8)
		{
			cpVect d = cpv(0, -0.1);
            cpBodySetPosition(controller, cpBodyGetPosition(controller) + d);
		}
		else if ((int)key == 9)
		{
			cpVect d = cpv(0, 0.1);
            cpBodySetPosition(controller, cpBodyGetPosition(controller) + d);
		}
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