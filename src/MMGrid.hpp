#define _CRT_SECURE_NO_WARNINGS

#include <iostream>

#include <igl/opengl/glfw/Viewer.h>
#include "external/Chipmunk2D/include/chipmunk/chipmunk.h"

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
    cpFloat linkMass;
    cpFloat bevel;
    vector<cpBody *> rowLinks, colLinks, crossLinks;
    vector<cpShape *> rowLinkShapes, colLinkShapes, crossLinkShapes;
    vector<cpConstraint *> constraints;
    MatrixX2d vertices;
    MatrixX2i edges;
    MatrixX3d pointColors;
    MatrixX3d edgeColors;
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

    cpShape *makeLinkShape(cpBody *body, cpVect posA, cpVect posB)
    {
        cpVect a = cpvzero;
        cpVect b = posB - posA;
        cpShape *shape = cpSpaceAddShape(space, cpSegmentShapeNew(body, a, b, bevel));
        cpShapeSetFriction(shape, 0.8);
        cpShapeSetElasticity(shape, 0.01);
        return shape;
    }

    void setupSimStructures();
    void removeSimStructures();
    void setupSpace()
    {
        cpVect gravity = cpv(0, -9.8);
        cpSpaceSetGravity(space, gravity);

        cpShape *ground = cpSegmentShapeNew(cpSpaceGetStaticBody(space), cpv(-10, 0), cpv(10, 0), 0);
        cpShapeSetElasticity(ground, 1);
        cpShapeSetFriction(ground, 1);
        cpSpaceAddShape(space, ground);
    }
    void updateVertices();
    void updateEdges();

public:
    bool changingStructure = false;
    MMGrid(int rows, int cols, cpFloat linkMass, cpFloat bevel, vector<int> cells);
    ~MMGrid();
    void render(igl::opengl::glfw::Viewer *viewer, int selected_cell);
    void render(igl::opengl::glfw::Viewer viewer, int selected_cell);
    void update(cpFloat dt);
    void setCells(int rows, int cols, vector<int> cells);
    void applyForce(int direction, int selected_cell);
    cpFloat getLinkMass() {return linkMass;};
    void setLinkMass(cpFloat linkMass) {
        this->linkMass = linkMass;
        setCells(rows, cols, cells);
    };
    cpFloat getBevel() {return bevel;};
    void setBevel(cpFloat linkMass) {
        this->bevel = bevel;
        setCells(rows, cols, cells);
    };
};