#include "MMGrid.hpp"

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
    rowLinkShapes.resize(numRowLinks());
    colLinkShapes.resize(numColLinks());
    constraints.reserve(numConstraints());

    cpVect rowBevOffset = cpv(bevel * SQRT_2, 0), colBevOffset = cpv(0, bevel * SQRT_2);
    rowBevOffset = rowBevOffset * shrink_factor;
    colBevOffset = colBevOffset * shrink_factor;
    cpVect bl = cpv(0, bevel * 2);

    // rowlinks
    for (int i = 0; i < numRowLinks(); i++)
    {
        int joint_index = (i / cols) * (jointCols()) + (i % cols);
        cpVect posA = bl + getJointOffset(joint_index) + rowBevOffset, posB = bl + getJointOffset(joint_index + 1) - rowBevOffset;
        cpBody *b = makeLinkBody(posA, posB);
        cpShape *s = makeLinkShape(b, posA, posB);
        rowLinks[i] = b;
        rowLinkShapes[i] = s;
    }
    // colLinks
    for (int i = 0; i < numColLinks(); i++)
    {
        int joint_index = i;
        cpVect posA = bl + getJointOffset(joint_index) + colBevOffset, posB = bl + getJointOffset(joint_index + jointCols()) - colBevOffset;
        cpBody *b = makeLinkBody(posA, posB);
        cpShape *s = makeLinkShape(b, posA, posB);
        colLinks[i] = b;
        colLinkShapes[i] = s;
    }
    // crossLinks
    int crossLinkIndex = 0;
    for (int i = 0; i < rows * cols; i++)
        if (cells[i] == 1)
        {
            int joint_index = (i / cols) * (jointCols()) + (i % cols);
            int a_joint_index = (i / cols) * (jointCols() + 1) + (i % cols);
            cpVect posA1 = bl + getJointOffset(joint_index), posB1 = bl + getJointOffset(a_joint_index + 1);
            cpVect posA2 = bl + getJointOffset(joint_index + 1), posB2 = bl + getJointOffset(a_joint_index);
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
            cpConstraint *pivot1 = cpPivotJointNew(a_prev_col, b1, bl + getJointOffset(joint_index));
            cpConstraint *pivot2 = cpPivotJointNew(a_next_col, b1, bl + getJointOffset(a_joint_index + 1));
            cpConstraint *pivot3 = cpPivotJointNew(row_current, b2, bl + getJointOffset(joint_index + 1));
            cpConstraint *pivot4 = cpPivotJointNew(row_next, b2, bl + getJointOffset(a_joint_index));
            cpSpaceAddConstraint(space, pivot1);
            cpSpaceAddConstraint(space, pivot2);
            cpSpaceAddConstraint(space, pivot3);
            cpSpaceAddConstraint(space, pivot4);
            constraints.push_back(pivot1);
            constraints.push_back(pivot2);
            constraints.push_back(pivot3);
            constraints.push_back(pivot4);
            // crossLinkShapes[i] = s;
        }

    // make pivot constraints
    // first row
    for (int i = 0; i < cols; i++)
    {
        cpBody *row_current = rowLinks[i];
        cpBody *prev_col = colLinks[i];
        cpBody *next_col = colLinks[i + 1];
        cpConstraint *pivot1 = cpPivotJointNew(prev_col, row_current, bl + getJointOffset(i));
        cpConstraint *pivot2 = cpPivotJointNew(next_col, row_current, bl + getJointOffset(i + 1));
        cpSpaceAddConstraint(space, pivot1);
        cpSpaceAddConstraint(space, pivot2);
        constraints.push_back(pivot1);
        constraints.push_back(pivot2);
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
        cpConstraint *pivot1 = cpPivotJointNew(prev_col, row_current, bl + getJointOffset(joint_idx));
        cpConstraint *pivot2 = cpPivotJointNew(next_col, row_current, bl + getJointOffset(joint_idx + 1));
        cpSpaceAddConstraint(space, pivot1);
        cpSpaceAddConstraint(space, pivot2);
        constraints.push_back(pivot1);
        constraints.push_back(pivot2);
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
        cpConstraint *pivot1 = cpPivotJointNew(a_prev_col, row_current, bl + getJointOffset(joint_idx));
        cpConstraint *pivot2 = cpPivotJointNew(a_next_col, row_current, bl + getJointOffset(joint_idx + 1));
        cpConstraint *pivot3 = cpPivotJointNew(b_prev_col, row_current, bl + getJointOffset(joint_idx));
        cpConstraint *pivot4 = cpPivotJointNew(b_next_col, row_current, bl + getJointOffset(joint_idx + 1));
        cpSpaceAddConstraint(space, pivot1);
        cpSpaceAddConstraint(space, pivot2);
        cpSpaceAddConstraint(space, pivot3);
        cpSpaceAddConstraint(space, pivot4);
        constraints.push_back(pivot1);
        constraints.push_back(pivot2);
        constraints.push_back(pivot3);
        constraints.push_back(pivot4);
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
        constraints.push_back(dampedRotarySpring1);
        constraints.push_back(dampedRotarySpring2);
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
        constraints.push_back(dampedRotarySpring1);
        constraints.push_back(dampedRotarySpring2);
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
        constraints.push_back(dampedRotarySpring1);
        constraints.push_back(dampedRotarySpring2);
        constraints.push_back(dampedRotarySpring3);
        constraints.push_back(dampedRotarySpring4);
    }
}

void MMGrid::updateVertices()
{
    cpVect rowBevOffset = cpv(bevel * SQRT_2, 0);
    rowBevOffset = rowBevOffset * shrink_factor;

    for (int i = 0; i < numRowLinks(); i++)
    {
        int row_i = i / cols;
        int row_offset = row_i;
        cpVect pos = cpBodyGetPosition(rowLinks[i]);
        cpVect rot = cpBodyGetRotation(rowLinks[i]);
        cpVect posA = cpvrotate(cpvzero - rowBevOffset, rot) + pos;
        // std::cout << posA.x << "," << posA.y << " " << posB.x << "," << posB.y << std::endl;
        vertices.row(i + row_offset) = (Vector2d() << posA.x, posA.y).finished();
        if ((i + 1) % cols == 0)
        {
            cpShape *shape = rowLinkShapes[i];
            cpVect posB = cpvrotate(cpSegmentShapeGetB(shape) + rowBevOffset, rot) + pos;
            vertices.row(i + row_offset + 1) = (Vector2d() << posB.x, posB.y).finished();
        }
    }
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

void MMGrid::render(igl::opengl::glfw::Viewer *viewer, int selected_cell)
{
    while (changingStructure)
    {
        cout << "Waiting for finish changing structure..." << endl;
    }
    pointColors = MatrixXd::Zero(vertices.rows(), 3);
    edgeColors = MatrixXd::Zero(edges.rows(), 3);

    int edge_index = 0;

    bool u_selection_done = false;
    bool r_selection_done = false;
    // update edge colors
    for (int i = 0; i < rows * cols; i++)
    {
        if (i == selected_cell)
            edgeColors.row(edge_index) = (Vector3d() << 1, 0, 0).finished();
        else if (!u_selection_done && i == (selected_cell + cols))
        {
            edgeColors.row(edge_index) = (Vector3d() << 1, 0, 0).finished();
            u_selection_done = true;
        }
        edge_index++;
        if (i == selected_cell)
            edgeColors.row(edge_index) = (Vector3d() << 1, 0, 0).finished();
        else if (!r_selection_done && i == (selected_cell + 1))
        {
            edgeColors.row(edge_index) = (Vector3d() << 1, 0, 0).finished();
            r_selection_done = true;
        }
        edge_index++;
        if ((i + 1) % cols == 0)
        {
            if (i == selected_cell)
            {
                edgeColors.row(edge_index) = (Vector3d() << 1, 0, 0).finished();
                r_selection_done = true;
            }
            edge_index++;
        }
        if (i >= (cols * (rows - 1)))
        {
            if (i == selected_cell)
            {
                edgeColors.row(edge_index) = (Vector3d() << 1, 0, 0).finished();
                u_selection_done = true;
            }
            edge_index++;
        }
        if (cells[i] == 1)
        {
            edgeColors.row(edge_index) = (Vector3d() << 0, 0, 1).finished();
            edge_index++;
            edgeColors.row(edge_index) = (Vector3d() << 0, 0, 1).finished();
            edge_index++;
        }
    }

    MatrixX3d points = MatrixXd::Zero(vertices.rows(), 3);
    points << vertices, MatrixXd::Zero(vertices.rows(), 1);
    viewer->data().clear();
    viewer->data().set_points(points, pointColors);
    viewer->data().set_edges(points, edges, edgeColors);
    viewer->data().set_mesh(mesh.first, mesh.second);
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
    // for (cpConstraint *c : constraints)
    // {
    //     if (cpSpaceContainsConstraint(space, c))
    //         cpSpaceRemoveConstraint(space, c);
    //     else
    //         cout << "Say sum'n" << endl;

    //     cpConstraintFree(c);
    // }
    // for (cpShape *l : rowLinkShapes)
    // {
    //     if (cpSpaceContainsShape(space, l))
    //         cpSpaceRemoveShape(space, l);
    //     else
    //         cout << "Say sum'n" << endl;

    //     cpShapeFree(l);
    // }
    // for (cpShape *l : colLinkShapes)
    // {
    //     if (cpSpaceContainsShape(space, l))
    //         cpSpaceRemoveShape(space, l);
    //     else
    //         cout << "Say sum'n" << endl;

    //     cpShapeFree(l);
    // }
    // for (cpBody *l : rowLinks)
    // {
    //     if (cpSpaceContainsBody(space, l))
    //         cpSpaceRemoveBody(space, l);
    //     else
    //         cout << "Say sum'n" << endl;

    //     cpBodyFree(l);
    // }
    // for (cpBody *l : colLinks)
    // {
    //     if (cpSpaceContainsBody(space, l))
    //         cpSpaceRemoveBody(space, l);
    //     else
    //         cout << "Say sum'n" << endl;

    //     cpBodyFree(l);
    // }
    // for (cpBody *l : crossLinks)
    // {
    //     if (cpSpaceContainsBody(space, l))
    //         cpSpaceRemoveBody(space, l);
    //     else
    //         cout << "Say sum'n" << endl;

    //     cpBodyFree(l);
    // }
    cpSpaceFree(space);
    rowLinks.clear();
    colLinks.clear();
    crossLinks.clear();
    rowLinkShapes.clear();
    colLinkShapes.clear();
    constraints.clear();
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