#define _CRT_SECURE_NO_WARNINGS

#include <iostream>

#include <igl/opengl/glfw/Viewer.h>
#include "chipmunk/chipmunk.h"
#include "rendering.hpp"
#include "ConstraintGraph.hpp"

#define SQRT_2 1.4142135623730950488016887242

using namespace std;
using namespace Eigen;

class MMGrid
{
private:
    int rows;
    int cols;
    vector<int> cells;
    cpSpace *space;
    cpFloat linkMass = 0.3;
    cpFloat bevel = .06;
    cpFloat stiffness = 0.6;
    cpFloat damping = 0.2;
    vector<cpBody *> rowLinks, colLinks, crossLinks, joints, controllers;
    vector<int> constrainedJoints;
    vector<cpConstraint *> controllerConstraints;
    MatrixX2d vertices;
    MatrixX2d targetVerts;
    MatrixX2i edges;
    MatrixX2i targetEdges;
    MatrixX3d pointColors;
    MatrixX3d edgeColors;
    cpVect bottomLeft;
    vector<cpVect> path;
    vector<int> targets;
    vector<vector<cpVect>> targetPaths;
    vector<int> anchors;
    int resolution = 6;
    int shrink_factor = 2;
    cpFloat frameTime = 0;
    int pointIndex = 0;
    std::pair<MatrixX3d, MatrixX3i> mesh;
    int jointRows() { return rows + 1; };
    int jointCols() { return cols + 1; };
    int numRowLinks() { return jointRows() * cols; };
    int numColLinks() { return jointCols() * rows; };
    int numCrossLinks()
    {
        int o = 0;
        for (int i = 0; i < rows * cols; i++)
            if (cells[i] == 1)
                o += 2;
        return o;
    };
    int numConstraints()
    {
        return (rows * cols * 4) + numCrossLinks() * 2;
    }

    cpVect getJointOffset(int index)
    {
        return cpv(index % jointCols(), index / jointCols());
    }

    cpBody *makeLinkBody(cpVect posA, cpVect posB)
    {
        cpVect a = cpvzero;
        cpVect b = posB - posA;
        cpFloat moment = cpMomentForSegment(linkMass, a, b, bevel);
        cpBody *body = cpSpaceAddBody(space, cpBodyNew(linkMass, moment));
        cpBodySetPosition(body, posA);
        return body;
    };

    cpBody *makeJointBody(cpVect pos) {
        cpBody *body = cpSpaceAddBody(space, cpBodyNew(linkMass / 10, INFINITY));
        cpBodySetPosition(body, pos);
        return body;
    }

    cpBody *makeControllerBody(cpVect pos) {
        cpBody *body = cpSpaceAddBody(space, cpBodyNewKinematic());
        cpBodySetPosition(body, pos);
        return body;
    }

    cpShape *makeLinkShape(cpBody *body, cpVect posA, cpVect posB)
    {
        cpVect a = cpvzero;
        cpVect b = posB - posA;
        cpShape *shape = cpSpaceAddShape(space, cpSegmentShapeNew(body, a, b, bevel));
        cpShapeSetFriction(shape, 0.999);
        cpShapeSetElasticity(shape, 0.00001);
        return shape;
    }

    void setupSimStructures();
    void removeSimStructures();
    void setupSpace()
    {
        cpVect gravity = cpv(0, -9.8);
        // cpSpaceSetGravity(space, gravity);

        cpShape *ground = cpSegmentShapeNew(cpSpaceGetStaticBody(space), cpv(-10, 0), cpv(10, 0), 0.05);
        cpShapeSetElasticity(ground, .05);
        cpShapeSetFriction(ground, 1);
        // cpSpaceAddShape(space, ground);
    }
    void updateVertices();
    void updateMesh();
    void updateEdges();
    void removeAllJointControllers();

public:
    bool changingStructure = false;
    MMGrid(int rows, int cols, vector<int> cells);
    MMGrid(const MMGrid& other) {
        rows = other.rows;
        cols = other.cols;
        cells = other.cells;
        path = other.path;
        targets = other.targets;
        targetPaths = other.targetPaths;
        anchors = other.anchors;
        vertices = MatrixXd::Zero(jointCols() * jointRows(), 2);
        edges = MatrixXi::Zero(numColLinks() + numCrossLinks() + numRowLinks(), 2);
        pointColors = MatrixXd::Zero(vertices.rows(), 3);
        edgeColors = MatrixXd::Zero(edges.rows(), 3);
        setupSimStructures();
        updateVertices();
        updateMesh();
        updateEdges();        
    }
    ~MMGrid();
    void render(igl::opengl::glfw::Viewer *viewer, int selected_cell, int selected_joint);
    void render(igl::opengl::glfw::Viewer viewer, int selected_cell);
    void update(cpFloat dt);
    void update_follow_path(cpFloat dt, int points_per_second);
    void setCells(int rows, int cols, vector<int> cells);
    void applyForce(int direction, int selected_cell);
    cpFloat getLinkMass() {return linkMass;};
    void setLinkMass(cpFloat linkMass) {
        this->linkMass = linkMass;
        setCells(rows, cols, cells);
    };
    cpFloat getBevel() {return bevel;};
    void setBevel(cpFloat bevel) {
        this->bevel = bevel;
        setCells(rows, cols, cells);
    };
    cpFloat getStiffness() {return stiffness;};
    void setStiffness(cpFloat stiffness) {
        this->stiffness = stiffness;
        setCells(rows, cols, cells);
    };
    cpFloat getDamping() {return damping;};
    void setDamping(cpFloat damping) {
        this->damping = damping;
        setCells(rows, cols, cells);
    };
    int getShrinkFactor() {return shrink_factor;};
    void setShrinkFactor(int shrink_factor) {
        this->shrink_factor = shrink_factor;
        setCells(rows, cols, cells);
    };
    int getRows() {return rows;};
    int getCols() {return cols;};
    vector<int> getCells() {return cells;}
    void addJointController(int jointIndex);
    void removeJointController(int jointIndex);
    cpVect getPos(int jointIndex);
    void moveController(int jointIndex, cpVect pos);
    bool isConstrained(int jointIndex);
    void setJointMaxForce(int jointIndex, cpFloat force);
    void loadFromFile(const std::string fname);
    double getPathError();
    double getCurrentError();
    void resetAnimation();
};