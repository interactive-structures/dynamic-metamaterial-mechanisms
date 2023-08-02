#define _USE_MATH_DEFINES
#include "MMGrid.hpp"

const int JOINT_MAX_FORCE = 100;

MMGrid::MMGrid(int rows, int cols, vector<int> cells)
{
    changingStructure = true;
    this->rows = rows;
    this->cols = cols;
    this->cells = cells;
    vertices = MatrixXd::Zero(jointCols() * jointRows(), 2);
    edges = MatrixXi::Zero(numColLinks() + numCrossLinks() + numRowLinks(), 2);
    pointColors = MatrixXd::Zero(vertices.rows(), 3);
    edgeColors = MatrixXd::Zero(edges.rows(), 3);
    setupSimStructures();
    updateVertices();
    updateMesh();
    updateEdges();
    changingStructure = false;
}

void MMGrid::setCells(int rows, int cols, vector<int> cells)
{
    changingStructure = true;
    removeSimStructures();
    resetAnimation();
    this->rows = rows;
    this->cols = cols;
    this->cells = cells;
    vertices = MatrixXd::Zero(jointCols() * jointRows(), 2);
    edges = MatrixXi::Zero(numColLinks() + numCrossLinks() + numRowLinks(), 2);
    setupSimStructures();
    updateVertices();
    updateMesh();
    updateEdges();
    changingStructure = false;
}

void MMGrid::setupSimStructures()
{
    this->space = cpSpaceNew();
    setupSpace();
    rowLinks.resize(numRowLinks());
    colLinks.resize(numColLinks());
    crossLinks.resize(numCrossLinks());
    joints.clear();
    joints.reserve(jointRows() * jointCols());
    controllers.clear();
    controllers.reserve(jointRows() * jointCols());
    constrainedJoints.clear();
    removeAllJointControllers();
    controllerConstraints.clear();
    controllerConstraints.reserve(jointRows() * jointCols());

    cpVect rowBevOffset = cpv(bevel * SQRT_2, 0), colBevOffset = cpv(0, bevel * SQRT_2);
    rowBevOffset = rowBevOffset * shrink_factor;
    colBevOffset = colBevOffset * shrink_factor;
    bottomLeft = cpv(0, 0);

    // rowlinks
    for (int i = 0; i < numRowLinks(); i++)
    {
        int joint_index = (i / cols) * (jointCols()) + (i % cols);
        cpVect posA = bottomLeft + getJointOffset(joint_index) + rowBevOffset;
        cpVect posB = bottomLeft + getJointOffset(joint_index + 1) - rowBevOffset;
        cpBody *b = makeLinkBody(posA, posB);
        cpShape *s = makeLinkShape(b, posA, posB);
        rowLinks[i] = b;
    }
    // colLinks
    for (int i = 0; i < numColLinks(); i++)
    {
        int joint_index = i;
        cpVect posA = bottomLeft + getJointOffset(joint_index) + colBevOffset;
        cpVect posB = bottomLeft + getJointOffset(joint_index + jointCols()) - colBevOffset;
        cpBody *b = makeLinkBody(posA, posB);
        cpShape *s = makeLinkShape(b, posA, posB);
        colLinks[i] = b;
    }
    // crossLinks
    int crossLinkIndex = 0;
    for (int i = 0; i < rows * cols; i++)
        if (cells[i] == 1)
        {
            int joint_index = (i / cols) * (jointCols()) + (i % cols);
            int a_joint_index = (i / cols) * (jointCols() + 1) + (i % cols);
            cpVect posA1 = bottomLeft + getJointOffset(joint_index), posB1 = bottomLeft + getJointOffset(a_joint_index + 1);
            cpVect posA2 = bottomLeft + getJointOffset(joint_index + 1), posB2 = bottomLeft + getJointOffset(a_joint_index);
            cpBody *b1 = makeLinkBody(posA1, posB1), *b2 = makeLinkBody(posA2, posB2);
            crossLinks[crossLinkIndex] = b1;
            crossLinkIndex++;
            crossLinks[crossLinkIndex] = b2;
            crossLinkIndex++;

            int row_i = i / (cols);
            int col_i = i % (cols);

            int a_col_prev_idx = (row_i) * (cols + 1) + col_i;
            int a_col_next_idx = a_col_prev_idx + 1;

            cpBody *row_current = rowLinks[i];
            cpBody *row_next = rowLinks[i + cols];
            cpBody *a_prev_col = colLinks[a_col_prev_idx];
            cpBody *a_next_col = colLinks[a_col_next_idx];
            cpConstraint *pivot1 = cpPivotJointNew(a_prev_col, b1, bottomLeft + getJointOffset(joint_index));
            cpConstraint *pivot2 = cpPivotJointNew(a_next_col, b1, bottomLeft + getJointOffset(a_joint_index + 1));
            cpConstraint *pivot3 = cpPivotJointNew(row_current, b2, bottomLeft + getJointOffset(joint_index + 1));
            cpConstraint *pivot4 = cpPivotJointNew(row_next, b2, bottomLeft + getJointOffset(a_joint_index));
            cpSpaceAddConstraint(space, pivot1);
            cpSpaceAddConstraint(space, pivot2);
            cpSpaceAddConstraint(space, pivot3);
            cpSpaceAddConstraint(space, pivot4);
        }

    // make pivot constraints
    // first row
    for (int i = 0; i < cols; i++)
    {
        cpBody *row_current = rowLinks[i];
        cpBody *prev_col = colLinks[i];
        cpBody *next_col = colLinks[i + 1];
        cpConstraint *pivot1 = cpPivotJointNew(prev_col, row_current, bottomLeft + getJointOffset(i));
        cpConstraint *pivot2 = cpPivotJointNew(next_col, row_current, bottomLeft + getJointOffset(i + 1));
        cpSpaceAddConstraint(space, pivot1);
        cpSpaceAddConstraint(space, pivot2);
    }
    // top row
    for (int i = rows * cols; i < (jointRows()) * cols; i++)
    {
        int row_i = i / cols;
        int col_i = i % cols;
        int col_prev_idx = (row_i - 1) * (jointCols()) + col_i;
        int col_next_idx = col_prev_idx + 1;
        int joint_idx = i / cols * (jointCols()) + (i % cols);
        cpBody *row_current = rowLinks[i];
        cpBody *prev_col = colLinks[col_prev_idx];
        cpBody *next_col = colLinks[col_next_idx];
        cpConstraint *pivot1 = cpPivotJointNew(prev_col, row_current, bottomLeft + getJointOffset(joint_idx));
        cpConstraint *pivot2 = cpPivotJointNew(next_col, row_current, bottomLeft + getJointOffset(joint_idx + 1));
        cpSpaceAddConstraint(space, pivot1);
        cpSpaceAddConstraint(space, pivot2);
    }
    // interior rows
    for (int i = cols; i < rows * cols; i++)
    {
        int row_i = i / (cols);
        int col_i = i % (cols);
        int b_col_prev_idx = (row_i - 1) * (cols + 1) + col_i;
        int b_col_next_idx = b_col_prev_idx + 1;
        int a_col_prev_idx = (row_i) * (cols + 1) + col_i;
        int joint_idx = i / (cols) * (cols + 1) + (i % (cols));
        int a_col_next_idx = a_col_prev_idx + 1;
        cpBody *row_current = rowLinks[i];
        cpBody *b_prev_col = colLinks[b_col_prev_idx];
        cpBody *b_next_col = colLinks[b_col_next_idx];
        cpBody *a_prev_col = colLinks[a_col_prev_idx];
        cpBody *a_next_col = colLinks[a_col_next_idx];
        cpConstraint *pivot1 = cpPivotJointNew(a_prev_col, row_current, bottomLeft + getJointOffset(joint_idx));
        cpConstraint *pivot2 = cpPivotJointNew(a_next_col, row_current, bottomLeft + getJointOffset(joint_idx + 1));
        cpConstraint *pivot3 = cpPivotJointNew(b_prev_col, row_current, bottomLeft + getJointOffset(joint_idx));
        cpConstraint *pivot4 = cpPivotJointNew(b_next_col, row_current, bottomLeft + getJointOffset(joint_idx + 1));
        cpSpaceAddConstraint(space, pivot1);
        cpSpaceAddConstraint(space, pivot2);
        cpSpaceAddConstraint(space, pivot3);
        cpSpaceAddConstraint(space, pivot4);
    }

    // make dampedRotarySpring constraints
    // first row
    for (int i = 0; i < cols; i++)
    {
        cpBody *row_current = rowLinks[i];
        cpBody *prev_col = colLinks[i];
        cpBody *next_col = colLinks[i + 1];
        cpConstraint *dampedRotarySpring1 = cpDampedRotarySpringNew(prev_col, row_current, 0, stiffness, damping);
        cpConstraint *dampedRotarySpring2 = cpDampedRotarySpringNew(next_col, row_current, 0, stiffness, damping);
        cpSpaceAddConstraint(space, dampedRotarySpring1);
        cpSpaceAddConstraint(space, dampedRotarySpring2);
    }
    // top row
    for (int i = rows * cols; i < (jointRows()) * cols; i++)
    {
        int row_i = i / cols;
        int col_i = i % cols;
        int col_prev_idx = (row_i - 1) * (jointCols()) + col_i;
        int col_next_idx = col_prev_idx + 1;
        int joint_idx = i / cols * (jointCols()) + (i % cols);
        cpBody *row_current = rowLinks[i];
        cpBody *prev_col = colLinks[col_prev_idx];
        cpBody *next_col = colLinks[col_next_idx];
        cpConstraint *dampedRotarySpring1 = cpDampedRotarySpringNew(prev_col, row_current, 0, stiffness, damping);
        cpConstraint *dampedRotarySpring2 = cpDampedRotarySpringNew(next_col, row_current, 0, stiffness, damping);
        cpSpaceAddConstraint(space, dampedRotarySpring1);
        cpSpaceAddConstraint(space, dampedRotarySpring2);
    }
    // interior rows
    for (int i = cols; i < rows * cols; i++)
    {
        int row_i = i / (cols);
        int col_i = i % (cols);
        int b_col_prev_idx = (row_i - 1) * (cols + 1) + col_i;
        int b_col_next_idx = b_col_prev_idx + 1;
        int a_col_prev_idx = (row_i) * (cols + 1) + col_i;
        int joint_idx = i / (cols) * (cols + 1) + (i % (cols));
        int a_col_next_idx = a_col_prev_idx + 1;
        cpBody *row_current = rowLinks[i];
        cpBody *b_prev_col = colLinks[b_col_prev_idx];
        cpBody *b_next_col = colLinks[b_col_next_idx];
        cpBody *a_prev_col = colLinks[a_col_prev_idx];
        cpBody *a_next_col = colLinks[a_col_next_idx];
        cpConstraint *dampedRotarySpring1 = cpDampedRotarySpringNew(a_prev_col, row_current, 0, stiffness, damping);
        cpConstraint *dampedRotarySpring2 = cpDampedRotarySpringNew(a_next_col, row_current, 0, stiffness, damping);
        cpConstraint *dampedRotarySpring3 = cpDampedRotarySpringNew(b_prev_col, row_current, 0, stiffness, damping);
        cpConstraint *dampedRotarySpring4 = cpDampedRotarySpringNew(b_next_col, row_current, 0, stiffness, damping);
        cpSpaceAddConstraint(space, dampedRotarySpring1);
        cpSpaceAddConstraint(space, dampedRotarySpring2);
        cpSpaceAddConstraint(space, dampedRotarySpring3);
        cpSpaceAddConstraint(space, dampedRotarySpring4);
    }

    // make joint bodies + constraints + controller bodies
    // first row
    for (int i = 0; i < cols; i++)
    {
        cpBody *row_current = rowLinks[i];
        cpBody *prev_col = colLinks[i];
        cpBody *next_col = colLinks[i + 1];
        cpBody *joint = makeJointBody(bottomLeft + getJointOffset(i));
        cpBody *controller = makeControllerBody(bottomLeft + getJointOffset(i));
        joints.push_back(joint);
        controllers.push_back(controller);
        cpConstraint *pivot1 = cpPivotJointNew(joint, row_current, bottomLeft + getJointOffset(i));
        cpConstraint *pivot2 = cpPivotJointNew(joint, prev_col, bottomLeft + getJointOffset(i));
        cpConstraint *controlPivot = cpPivotJointNew(joint, controller, bottomLeft + getJointOffset(i));
        cpSpaceAddConstraint(space, pivot1);
        cpSpaceAddConstraint(space, pivot2);
        controllerConstraints.push_back(controlPivot);
        if (i == cols - 1)
        {
            cpBody *joint = makeJointBody(bottomLeft + getJointOffset(i + 1));
            cpBody *controller = makeControllerBody(bottomLeft + getJointOffset(i + 1));
            joints.push_back(joint);
            controllers.push_back(controller);
            cpConstraint *pivot1 = cpPivotJointNew(joint, row_current, bottomLeft + getJointOffset(i + 1));
            cpConstraint *pivot2 = cpPivotJointNew(joint, next_col, bottomLeft + getJointOffset(i + 1));
            cpConstraint *controlPivot = cpPivotJointNew(joint, controller, bottomLeft + getJointOffset(i + 1));
            cpSpaceAddConstraint(space, pivot1);
            cpSpaceAddConstraint(space, pivot2);
            controllerConstraints.push_back(controlPivot);
        }
    }

    // interior rows
    for (int i = cols; i < rows * cols; i++)
    {
        int row_i = i / (cols);
        int col_i = i % (cols);
        int b_col_prev_idx = (row_i - 1) * (cols + 1) + col_i;
        int b_col_next_idx = b_col_prev_idx + 1;
        int a_col_prev_idx = (row_i) * (cols + 1) + col_i;
        int joint_idx = i / (cols) * (cols + 1) + (i % (cols));
        int a_col_next_idx = a_col_prev_idx + 1;
        cpBody *row_current = rowLinks[i];
        cpBody *b_prev_col = colLinks[b_col_prev_idx];
        cpBody *b_next_col = colLinks[b_col_next_idx];
        cpBody *a_prev_col = colLinks[a_col_prev_idx];
        cpBody *a_next_col = colLinks[a_col_next_idx];
        cpBody *joint = makeJointBody(bottomLeft + getJointOffset(joint_idx));
        cpBody *controller = makeControllerBody(bottomLeft + getJointOffset(joint_idx));
        joints.push_back(joint);
        controllers.push_back(controller);
        cpConstraint *pivot1 = cpPivotJointNew(a_prev_col, joint, bottomLeft + getJointOffset(joint_idx));
        cpConstraint *pivot2 = cpPivotJointNew(joint, row_current, bottomLeft + getJointOffset(joint_idx));
        cpConstraint *pivot3 = cpPivotJointNew(b_prev_col, joint, bottomLeft + getJointOffset(joint_idx));
        cpConstraint *pivot4 = cpPivotJointNew(joint, row_current, bottomLeft + getJointOffset(joint_idx));
        cpConstraint *controlPivot = cpPivotJointNew(joint, controller, bottomLeft + getJointOffset(joint_idx));
        cpSpaceAddConstraint(space, pivot1);
        cpSpaceAddConstraint(space, pivot2);
        cpSpaceAddConstraint(space, pivot3);
        cpSpaceAddConstraint(space, pivot4);
        controllerConstraints.push_back(controlPivot);
        // cpConstraintSetMaxForce(controlPivot, 20);
        if ((i + 1) % cols == 0)
        {
            cpBody *joint = makeJointBody(bottomLeft + getJointOffset(joint_idx + 1));
            cpBody *controller = makeControllerBody(bottomLeft + getJointOffset(joint_idx + 1));
            joints.push_back(joint);
            controllers.push_back(controller);
            cpConstraint *pivot1 = cpPivotJointNew(a_next_col, joint, bottomLeft + getJointOffset(joint_idx + 1));
            cpConstraint *pivot2 = cpPivotJointNew(joint, row_current, bottomLeft + getJointOffset(joint_idx + 1));
            cpConstraint *pivot3 = cpPivotJointNew(b_next_col, joint, bottomLeft + getJointOffset(joint_idx + 1));
            cpConstraint *pivot4 = cpPivotJointNew(joint, row_current, bottomLeft + getJointOffset(joint_idx + 1));
            cpConstraint *controlPivot = cpPivotJointNew(joint, controller, bottomLeft + getJointOffset(joint_idx + 1));
            cpSpaceAddConstraint(space, pivot1);
            cpSpaceAddConstraint(space, pivot2);
            cpSpaceAddConstraint(space, pivot3);
            cpSpaceAddConstraint(space, pivot4);
            controllerConstraints.push_back(controlPivot);
        }
    }

    // top row
    for (int i = rows * cols; i < (jointRows()) * cols; i++)
    {
        int row_i = i / cols;
        int col_i = i % cols;
        int col_prev_idx = (row_i - 1) * (jointCols()) + col_i;
        int col_next_idx = col_prev_idx + 1;
        int joint_idx = i / cols * (jointCols()) + (i % cols);
        cpBody *row_current = rowLinks[i];
        cpBody *prev_col = colLinks[col_prev_idx];
        cpBody *next_col = colLinks[col_next_idx];
        cpBody *joint = makeJointBody(bottomLeft + getJointOffset(joint_idx));
        cpBody *controller = makeControllerBody(bottomLeft + getJointOffset(joint_idx));
        joints.push_back(joint);
        controllers.push_back(controller);
        cpConstraint *pivot1 = cpPivotJointNew(joint, row_current, bottomLeft + getJointOffset(joint_idx));
        cpConstraint *pivot2 = cpPivotJointNew(joint, prev_col, bottomLeft + getJointOffset(joint_idx));
        cpConstraint *controlPivot = cpPivotJointNew(joint, controller, bottomLeft + getJointOffset(joint_idx));
        cpSpaceAddConstraint(space, pivot1);
        cpSpaceAddConstraint(space, pivot2);
        controllerConstraints.push_back(controlPivot);
        if (i == (jointRows()) * cols - 1)
        {
            cpBody *joint = makeJointBody(bottomLeft + getJointOffset(joint_idx + 1));
            cpBody *controller = makeControllerBody(bottomLeft + getJointOffset(joint_idx + 1));
            joints.push_back(joint);
            controllers.push_back(controller);
            cpConstraint *pivot1 = cpPivotJointNew(joint, row_current, bottomLeft + getJointOffset(joint_idx + 1));
            cpConstraint *pivot2 = cpPivotJointNew(joint, next_col, bottomLeft + getJointOffset(joint_idx + 1));
            cpConstraint *controlPivot = cpPivotJointNew(joint, controller, bottomLeft + getJointOffset(joint_idx + 1));
            cpSpaceAddConstraint(space, pivot1);
            cpSpaceAddConstraint(space, pivot2);
            controllerConstraints.push_back(controlPivot);
        }
    }
}

bool MMGrid::isConstrained(int jointIndex)
{
    return find(constrainedJoints.begin(), constrainedJoints.end(), jointIndex) != constrainedJoints.end();
}

void MMGrid::updateVertices()
{
    cpVect rowBevOffset = cpv(bevel * SQRT_2, 0);
    rowBevOffset = rowBevOffset * shrink_factor;
    // cout << "r * c: " << jointRows() * jointCols() << ", js: " << joints.size() << ", cs: " << controllers.size() << ", vs: " << vertices.rows() << endl;

    for (int i = 0; i < jointRows() * jointCols(); i++)
    {
        cpVect pos = cpBodyGetPosition(joints[i]);
        if (isConstrained(i))
        {
            cpVect posC = cpBodyGetPosition(controllers[i]);
            vertices.row(i) = (Vector2d() << posC.x, posC.y).finished();
        }
        else
        {
            vertices.row(i) = (Vector2d() << pos.x, pos.y).finished();
            cpBody *controller = controllers[i];
            cpBodySetPosition(controller, pos);
        }
    }

    // for (int i = 0; i < numRowLinks(); i++)
    // {
    //     int row_i = i / cols;
    //     int row_offset = row_i;
    //     cpVect pos = cpBodyGetPosition(rowLinks[i]);
    //     cpVect rot = cpBodyGetRotation(rowLinks[i]);
    //     cpVect posA = cpvrotate(cpvzero - rowBevOffset, rot) + pos;
    //     // std::cout << posA.x << "," << posA.y << " " << posB.x << "," << posB.y << std::endl;
    //     vertices.row(i + row_offset) = (Vector2d() << posA.x, posA.y).finished();
    //     if ((i + 1) % cols == 0)
    //     {
    //         cpShape *shape = rowLinkShapes[i];
    //         cpVect posB = cpvrotate(cpSegmentShapeGetB(shape) + rowBevOffset, rot) + pos;
    //         vertices.row(i + row_offset + 1) = (Vector2d() << posB.x, posB.y).finished();
    //     }
    // }
}

void MMGrid::updateMesh()
{

    cpVect rowBevOffset = cpv(bevel * SQRT_2, 0);
    rowBevOffset = rowBevOffset * shrink_factor;

    std::vector<std::pair<MatrixX3d, MatrixX3i>> meshes;

    MatrixX3d groundV = MatrixX3d::Zero(4, 3);
    groundV << -10, 0, -10,
        -10, 0, 10,
        10, 0, -10,
        10, 0, 10;
    MatrixX3i groundF = MatrixX3i::Zero(2, 3);
    groundF << 0, 1, 2,
        2, 1, 3;
    meshes.push_back(std::make_pair(groundV, groundF));

    for (int i = 0; i < numRowLinks(); i++)
    {
        cpVect pos = cpBodyGetPosition(rowLinks[i]);
        cpVect rot = cpBodyGetRotation(rowLinks[i]);
        double rotation = cpvtoangle(rot);
        Vector3d base((double)pos.x, (double)pos.y, 0);
        std::pair<MatrixX3d, MatrixX3i> link = generateCapsule(base, bevel, 1 - 2 * (double)rowBevOffset.x, resolution, rotation - M_PI_2);
        meshes.push_back(link);
    }
    for (int i = 0; i < numColLinks(); i++)
    {
        cpVect pos = cpBodyGetPosition(colLinks[i]);
        cpVect rot = cpBodyGetRotation(colLinks[i]);
        double rotation = cpvtoangle(rot);
        Vector3d base((double)pos.x, (double)pos.y, 0);
        std::pair<MatrixX3d, MatrixX3i> link = generateCapsule(base, bevel, 1 - 2 * (double)rowBevOffset.x, resolution, rotation);
        meshes.push_back(link);
    }
    mesh = combineMeshes(meshes);
}

void MMGrid::updateMeshUnified()
{

    cpVect rowBevOffset = cpv(bevel * SQRT_2, 0);
    rowBevOffset = rowBevOffset * shrink_factor;

    std::vector<std::pair<MatrixX3d, MatrixX3i>> meshes;

    // MatrixX3d groundV = MatrixX3d::Zero(4, 3);
    // groundV << -10, 0, -10,
    //     -10, 0, 10,
    //     10, 0, -10,
    //     10, 0, 10;
    // MatrixX3i groundF = MatrixX3i::Zero(2, 3);
    // groundF << 0, 1, 2,
    //     2, 1, 3;
    // meshes.push_back(std::make_pair(groundV, groundF));

































    for (int i = 0; i < numRowLinks(); i++)
    {
        cpVect pos = cpBodyGetPosition(rowLinks[i]);
        cpVect rot = cpBodyGetRotation(rowLinks[i]);
        double rotation = cpvtoangle(rot);
        Vector3d base((double)pos.x, (double)pos.y, 0);
        std::pair<MatrixX3d, MatrixX3i> link = generateCapsule(base, bevel, 1 - 2 * (double)rowBevOffset.x, resolution, rotation - M_PI_2);
        meshes.push_back(link);
    }
    for (int i = 0; i < numColLinks(); i++)
    {
        cpVect pos = cpBodyGetPosition(colLinks[i]);
        cpVect rot = cpBodyGetRotation(colLinks[i]);
        double rotation = cpvtoangle(rot);
        Vector3d base((double)pos.x, (double)pos.y, 0);
        std::pair<MatrixX3d, MatrixX3i> link = generateCapsule(base, bevel, 1 - 2 * (double)rowBevOffset.x, resolution, rotation);
        meshes.push_back(link);
    }
    mesh = combineMeshes(meshes);
}

void MMGrid::updateEdges()
{
    // add edges to cells
    int edge_index = 0;
    for (int i = 0; i < rows * cols; i++)
    {
        int bl_joint_idx = i / (cols) * (cols + 1) + (i % (cols));
        int br_joint_idx = bl_joint_idx + 1;
        int ul_joint_idx = bl_joint_idx + (jointCols());
        int ur_joint_idx = ul_joint_idx + 1;
        edges.row(edge_index) = (Vector2i() << bl_joint_idx, br_joint_idx).finished();
        edge_index++;
        edges.row(edge_index) = (Vector2i() << bl_joint_idx, ul_joint_idx).finished();
        edge_index++;
        if ((i + 1) % cols == 0)
        {
            edges.row(edge_index) = (Vector2i() << br_joint_idx, ur_joint_idx).finished();
            edge_index++;
        }
        if (i >= (cols * (rows - 1)))
        {
            edges.row(edge_index) = (Vector2i() << ul_joint_idx, ur_joint_idx).finished();
            edge_index++;
        }
        if (cells[i] == 1)
        {
            edges.row(edge_index) = (Vector2i() << bl_joint_idx, ur_joint_idx).finished();
            edge_index++;
            edges.row(edge_index) = (Vector2i() << br_joint_idx, ul_joint_idx).finished();
            edge_index++;
        }
    }
}

void MMGrid::render(igl::opengl::glfw::Viewer *viewer, int selected_cell, int selected_joint)
{
    while (changingStructure)
    {
        cout << "Waiting for finish changing structure..." << endl;
    }
    pointColors = MatrixXd::Zero(vertices.rows(), 3);
    edgeColors = MatrixXd::Zero(edges.rows(), 3);

    pointColors.row(selected_joint) += (Vector3d() << 1, 0, 0).finished();
    for (int index : constrainedJoints)
    {
        pointColors.row(index) += (Vector3d() << 0, 0, 1).finished();
    }

    int edge_index = 0;

    bool u_selection_done = false;
    bool r_selection_done = false;
    // update edge colors
    for (int i = 0; i < rows * cols; i++)
    {
        if (i == selected_cell)
            edgeColors.row(edge_index) += (Vector3d() << 1, 0, 0).finished();
        else if (!u_selection_done && i == (selected_cell + cols))
        {
            edgeColors.row(edge_index) += (Vector3d() << 1, 0, 0).finished();
            u_selection_done = true;
        }
        edge_index++;
        if (i == selected_cell)
            edgeColors.row(edge_index) += (Vector3d() << 1, 0, 0).finished();
        else if (!r_selection_done && i == (selected_cell + 1))
        {
            edgeColors.row(edge_index) += (Vector3d() << 1, 0, 0).finished();
            r_selection_done = true;
        }
        edge_index++;
        if ((i + 1) % cols == 0)
        {
            if (i == selected_cell)
            {
                edgeColors.row(edge_index) += (Vector3d() << 1, 0, 0).finished();
                r_selection_done = true;
            }
            edge_index++;
        }
        if (i >= (cols * (rows - 1)))
        {
            if (i == selected_cell)
            {
                edgeColors.row(edge_index) += (Vector3d() << 1, 0, 0).finished();
                u_selection_done = true;
            }
            edge_index++;
        }
        if (cells[i] == 1)
        {
            edgeColors.row(edge_index) += (Vector3d() << 0, 0, 1).finished();
            edge_index++;
            edgeColors.row(edge_index) += (Vector3d() << 0, 0, 1).finished();
            edge_index++;
        }
    }

    MatrixX3d points = MatrixXd::Zero(vertices.rows(), 3);
    points << vertices, MatrixXd::Zero(vertices.rows(), 1);

    MatrixX3d edgePoints = MatrixXd::Zero(vertices.rows(), 3);
    MatrixX3d edgeColors2 = MatrixXd::Zero(edgeColors.rows(), 3);
    MatrixX2i edges2 = MatrixXi::Zero(edges.rows(), 2);
    if (targetVerts.rows() == 0)
    {
        edgePoints << points;
        edges2 << edges;
        edgeColors2 << edgeColors;
    }
    else
    {
        MatrixX2d intermediate = MatrixXd::Zero(vertices.rows() + targetVerts.rows(), 2);
        intermediate << vertices, targetVerts;
        edgePoints = MatrixXd::Zero(intermediate.rows(), 3);
        edgePoints << intermediate, MatrixXd::Zero(intermediate.rows(), 1);
        edges2 = MatrixXi::Zero(edges.rows() + targetEdges.rows(), 2);
        edges2 << edges, targetEdges + MatrixXi::Constant(targetEdges.rows(), 2, vertices.rows());
        edgeColors2 = MatrixXd::Zero(edges.rows() + targetEdges.rows(), 3);
        edgeColors2 << edgeColors, MatrixXd::Zero(targetEdges.rows(), 3);
    }
    viewer->data().clear();
    viewer->data().set_points(points, pointColors);
    viewer->data().set_edges(edgePoints, edges2, edgeColors2);
    viewer->data().set_mesh(mesh.first, mesh.second);
}

void MMGrid::resetAnimation() {
    pointIndex = 0;
    frameTime = 0;
}

vector<vector<double>> MMGrid::getAnglesFor(vector<int> cellIndices)
{
    vector<int> rowLinkIndices;
    vector<int> colLinkIndices;
    vector<vector<double>> out;
    for(int cellIndex : cellIndices) {
        rowLinkIndices.push_back(cellIndex);
        colLinkIndices.push_back(cellIndex - cellIndex / cols);
        out.push_back({});
    }
    int pathStepsPerSec = 3;
    double timeStep = 0.5 / 60;
    double totError = 0;
    double haltDelta = 1e-4;
    update_follow_path(timeStep, pathStepsPerSec);
    for(int i = 0; i < (rows + 1) * (cols + 1); i++) {
        if(!isConstrained(i)) {
            cpSpaceRemoveBody(space, joints[i]);
            cpSpaceRemoveBody(space, controllers[i]);
        } else {
            cout << i << " is constrained." << endl;
        }
    }
    for (int pathStep = 0; pathStep < targetPaths[0].size(); pathStep++)
    {
        int numIterations = 0;
        double curError, prevError = INT8_MAX;
        clock_t start, end;
        start = clock();
        while(pointIndex < pathStep) {
            update_follow_path(timeStep, pathStepsPerSec);
            numIterations++;
            curError = getCurrentError();
            if(abs(prevError - curError) < haltDelta) {
                pointIndex++;
                frameTime = 0;
            }
            prevError = curError;
        }
        for(int i = 0; i < cellIndices.size(); i++) {
            double angleDiff = cpvtoangle(cpBodyGetRotation(colLinks[colLinkIndices[i]])) + M_PI_2 - cpvtoangle(cpBodyGetRotation(rowLinks[rowLinkIndices[i]]));
            out[i].push_back(angleDiff);
        }
    }
    return out;
}

void MMGrid::writeConfig(string filePath)
{
    ofstream file(filePath);

    if (!file.good())
    {
        cout << "file " << filePath << " not found!" << endl;
        return;
    }
    file << std::endl;

    int pathLength = targetPaths.size() == 0? 0 : targetPaths[0].size();

    file << (rows + 1) * (cols + 1) << " " << rows * cols << " " << anchors.size() << " " << targetPaths.size() << " " << pathLength << std::endl;

    file << std::endl;
    file << std::endl;

    double x = bottomLeft.x, y = bottomLeft.y;

    for (int i = 0; i < (rows + 1) * (cols + 1); ++i)
    {
        file << x << " " << y << std::endl;
    }

    file << std::endl;
    file << std::endl;

    for (int x : anchors)
    {
        file << x << " ";
    }

    // shift = vertices[anchors.front()];
    //  anchors.pop_back();

    file << std::endl;
    file << std::endl;
    file << std::endl;

    int l_rows = -1, l_cols = -1;
    int prev_bottomleft = 0;
    vector<int> l_cells;

    int offset = 0;

    for (int i = 0; i < cells.size(); ++i)
    {
        if(cells[i] == 0) {
            file << "s";
        } else {
            file << "r";
        }

        if(i != 0 && i % cols == 0) offset++;

        file << " " << i + offset << " 0 0 0" << std::endl;
    }

    int numVertRows = 0;
    int numEdgeRows = 0;

    for (int c = 0; c < targets.size(); c++)
    {
        file << std::endl;
        file << std::endl;

        file << targets[c] << std::endl;

        for (int i = 0; i < targetPaths[c].size(); ++i)
        {
            double x = targetPaths[c][i].x, y = targetPaths[c][i].y;
            file << x << " " << y << std::endl;
        }
    }

    file.close();
}

void MMGrid::update(cpFloat dt)
{
    changingStructure = true;
    cpSpaceStep(space, dt);
    updateVertices();
    updateMesh();
    changingStructure = false;
}
MMGrid::~MMGrid()
{
    cout << "Destroying / freeing" << endl;
    removeSimStructures();
}
void MMGrid::removeSimStructures()
{
    cpSpaceFree(space);
    rowLinks.clear();
    colLinks.clear();
    crossLinks.clear();
}

void MMGrid::applyForce(int direction, int selected_cell)
{
    int i = selected_cell;
    int row_i = i / (cols);
    int col_i = i % (cols);

    int a_col_prev_idx = (row_i) * (cols + 1) + col_i;
    int a_col_next_idx = a_col_prev_idx + 1;

    cpBody *row_current = rowLinks[i];
    cpBody *row_next = rowLinks[i + cols];
    cpBody *a_prev_col = colLinks[a_col_prev_idx];
    cpBody *toApply;
    cpVect f;
    if (direction == 0)
    {
        toApply = row_next;
        f = cpv(1, 0);
    }
    else if (direction == 1)
    {
        toApply = row_next;
        f = cpv(-1, 0);
    }
    else if (direction == 2)
    {
        toApply = a_prev_col;
        f = cpv(0, 1);
    }
    else if (direction == 3)
    {
        toApply = a_prev_col;
        f = cpv(0, -1);
    }
    cpBodyApplyImpulseAtLocalPoint(toApply, f, cpvzero);
}

void MMGrid::addJointController(int jointIndex)
{
    if (isConstrained(jointIndex))
        return;
    constrainedJoints.push_back(jointIndex);
    cpSpaceAddConstraint(space, controllerConstraints[jointIndex]);
}
void MMGrid::removeJointController(int jointIndex)
{
    int pos = -1;
    for (int i = 0; i < constrainedJoints.size(); i++)
    {
        if (jointIndex == constrainedJoints[i])
        {
            cpSpaceRemoveConstraint(space, controllerConstraints[jointIndex]);
            pos = i;
        }
    }
    if (pos != -1)
    {
        constrainedJoints.erase(constrainedJoints.begin() + pos);
    }
}
void MMGrid::removeAllJointControllers()
{
    for (int jointIndex : constrainedJoints)
    {
        cpSpaceRemoveConstraint(space, controllerConstraints[jointIndex]);
    }
    controllerConstraints.clear();
}

cpVect MMGrid::getPos(int jointIndex)
{
    return cpBodyGetPosition(controllers[jointIndex]);
}

void MMGrid::setJointMaxForce(int jointIndex, cpFloat force)
{
    cpConstraintSetMaxForce(controllerConstraints[jointIndex], force);
}

void MMGrid::moveController(int jointIndex, cpVect pos)
{
    cpBodySetPosition(controllers[jointIndex], pos);
}

void MMGrid::update_follow_path(cpFloat dt, int points_per_second)
{
    cpFloat pps = 1.0 / (cpFloat)points_per_second;
    frameTime += dt;
    for (int anchorIndex : anchors)
    {
        addJointController(anchorIndex);
    }
    for (int i = 0; i < targetPaths.size(); i++)
    {
        int targetIndex = targets[i];
        vector<cpVect> targetPath = targetPaths[i];
        addJointController(targetIndex);
        setJointMaxForce(targetIndex, JOINT_MAX_FORCE);
        moveController(targetIndex, bottomLeft + targetPath[pointIndex]);
    }
    update(dt);
    if (targetPaths.size() > 0)
    {
        if (frameTime > pps)
        {
            pointIndex++;
            pointIndex %= targetPaths[0].size();
            frameTime = 0;
        }
    }
    else
    {
        // cout << "No target paths" << endl;
    }
}

void MMGrid::loadFromFile(const std::string fname)
{
    ifstream file(fname);

    if (!file.good())
    {
        cout << "file " << fname << " not found!" << endl;
        return;
    }
    char tmp[1024];

    file.getline(tmp, 1024);

    int nv, nc, na, nconstr, npath;

    file >> nv;
    file >> nc;
    file >> na;
    file >> nconstr;
    file >> npath;

    file.getline(tmp, 1024);
    file.getline(tmp, 1024);
    file.getline(tmp, 1024);

    cpFloat min_x = DBL_MIN, min_y = DBL_MIN;

    for (int i = 0; i < nv; ++i)
    {
        double x, y;
        file >> x;
        file >> y;

        if (x < min_x)
            min_x = x;
        if (y < min_y)
            min_y = y;
    }

    bottomLeft = cpv(min_x, min_y);

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

    int l_rows = -1, l_cols = -1;
    int prev_bottomleft = 0;
    vector<int> l_cells;

    for (int i = 0; i < nc; ++i)
    {
        char t;
        file >> t;

        int a, b, c, d;
        file >> a;
        file >> b;
        file >> c;
        file >> d;
        if (a - prev_bottomleft > 1 && l_cols == -1)
        {
            l_cols = a - 1;
        }
        prev_bottomleft = a;

        l_cells.push_back(t == 's' ? 0 : t == 'a' ? 2
                                                  : 1);
    }

    l_rows = prev_bottomleft / (l_cols + 1) + 1;

    cout << "READ IN " << l_rows << ", " << l_cols << endl;

    setCells(l_rows, l_cols, l_cells);

    int numVertRows = 0;
    int numEdgeRows = 0;

    for (int c = 0; c < nconstr; c++)
    {
        file.getline(tmp, 1024);
        file.getline(tmp, 1024);
        file.getline(tmp, 1024);

        int constr;
        file >> constr;

        targets.push_back(constr);

        for (int i = 0; i < npath; ++i)
        {
            double x, y;
            file >> x;
            file >> y;

            path.push_back(cpv(x, y));

            numVertRows++;
            numEdgeRows++;
        }

        targetPaths.push_back(path);
        path.clear();
    }

    targetVerts = MatrixXd::Zero(numVertRows, 2);
    targetEdges = MatrixXi::Zero(numEdgeRows, 2);

    int vert_index = 0;
    int edge_index = 0;
    for (auto tP : targetPaths)
    {
        int start_index = 0;
        for (auto pv : tP)
        {
            targetVerts.row(vert_index) = (Vector2d() << pv.x, pv.y).finished();
            targetEdges.row(edge_index) = (Vector2i() << vert_index, (vert_index + 1) % tP.size() + start_index).finished();
            vert_index++;
            edge_index++;
        }
    }

    file.close();
}

double MMGrid::getCurrentError() {
    double pointError = 0;
    for (int i = 0; i < targetPaths.size(); i++)
    {
        int targetIndex = targets[i];
        vector<cpVect> targetPath = targetPaths[i];
        cpVect posActual = cpBodyGetPosition(joints[targetIndex]);
        cpVect posTarget = targetPath[pointIndex];
        pointError += cpvdistsq(posActual, posTarget);
    }
    return pointError;
}

double MMGrid::getPathError()
{
    int pathStepsPerSec = 3;
    double timeStep = 0.5 / 60;
    double totError = 0;
    double haltDelta = 1e-4;
    update_follow_path(timeStep, pathStepsPerSec);
    for(int i = 0; i < (rows + 1) * (cols + 1); i++) {
        if(!isConstrained(i)) {
            cpSpaceRemoveBody(space, joints[i]);
            cpSpaceRemoveBody(space, controllers[i]);
        } else {
            cout << i << " is constrained." << endl;
        }
    }
    for (int pathStep = 0; pathStep < targetPaths[0].size(); pathStep++)
    {
        int numIterations = 0;
        double curError, prevError = INT8_MAX;
        clock_t start, end;
        start = clock();
        while(pointIndex < pathStep) {
            update_follow_path(timeStep, pathStepsPerSec);
            numIterations++;
            curError = getCurrentError();
            if(abs(prevError - curError) < haltDelta) {
                pointIndex++;
                frameTime = 0;
            }
            prevError = curError;
        }
        end = clock();
        cout << "For path step " << pathStep << " error is " << curError << " with " << numIterations << " iterations in " << double(end - start) / double(CLOCKS_PER_SEC) << " seconds." << endl;
        totError += curError;
    }
    cout << "Calculated Error: " << totError << endl;
    return totError;
}