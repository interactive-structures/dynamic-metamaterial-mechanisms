//  MetaGrid.cpp
//  main
//
//  Created by philipp on 31.01.18.
//

#include "MetaGrid.hpp"
#include <math.h>
#include <fstream>
#include <iostream>

#define PI 3.14159265

using namespace std;

MetaGrid::MetaGrid()
{
}

Cell::Cell(CellType _type, const int a, const int b, const int c, const int d)
    : type(_type), V{a, b, c, d}
{
}

vector<vector<int> > MetaGrid::connectedComponents(vector<vector<RelEdge> > &graph)
{
    vector<vector<int> > ret;
    const int n = graph.size();

    vector<char> visited(n, false);

    while (1)
    {
        int i = 0;
        for (; i < n; ++i)
            if (!visited[i])
                break;

        if (i == n)
            break;

        vector<int> comp{i};
        visited[i] = true;

        int pos = 0;
        while (pos < comp.size())
        {
            for (auto i : graph[comp[pos++]])
            {
                if (i.i != -1 && !visited[i.i])
                {
                    comp.push_back(i.i);
                    visited[i.i] = true;
                }
            }
        }

        ret.push_back(move(comp));
    }

    return ret;
}

Eigen::Matrix2d transformation(RelationType type)
{
    Eigen::Matrix2d m;
    m.setZero();

    switch (type)
    {
    case IDENTITY:
        m(0, 0) = 1;
        m(1, 1) = 1;
        break;

    case INVERTED:
        m(0, 0) = -1;
        m(1, 1) = -1;
        break;

    case ROT90:
        m(0, 1) = -1;
        m(1, 0) = 1;
        break;

    case ROT270:
        m(0, 1) = 1;
        m(1, 0) = -1;
        break;

    case ROT_PI_MINUS_THETA:
        cout << "Active cell found when none were expected." << endl;
        break;

    case ROT_2PI_MINUS_THETA:
        cout << "Active cell found when none were expected." << endl;
        break;

    case ROT_THETA:
        cout << "Active cell found when none were expected." << endl;
        break;

    case ROT_THETA_PLUS_PI:
        cout << "Active cell found when none were expected." << endl;
        break;

    default:
        cout << "Unknown RelationType" << endl;
        break;
    }

    return m;
}

Eigen::Matrix2d transformationActive(RelationType type, double angle)
{
    Eigen::Matrix2d m;
    m.setZero();

    switch (type)
    {
    case IDENTITY:
        m(0, 0) = 1;
        m(1, 1) = 1;
        break;

    case INVERTED:
        m(0, 0) = -1;
        m(1, 1) = -1;
        break;

    case ROT90:
        m(0, 1) = -1;
        m(1, 0) = 1;
        break;

    case ROT270:
        m(0, 1) = 1;
        m(1, 0) = -1;
        break;

    case ROT_PI_MINUS_THETA:
        m(0, 0) = cos(PI - angle);
        m(0, 1) = -sin(PI - angle);
        m(1, 0) = sin(PI - angle);
        m(1, 1) = cos(PI - angle);
        break;

    case ROT_2PI_MINUS_THETA:
        m(0, 0) = cos(2 * PI - angle);
        m(0, 1) = -sin(2 * PI - angle);
        m(1, 0) = sin(2 * PI - angle);
        m(1, 1) = cos(2 * PI - angle);
        break;

    case ROT_THETA:
        m(0, 0) = cos(angle);
        m(0, 1) = -sin(angle);
        m(1, 0) = sin(angle);
        m(1, 1) = cos(angle);
        break;

    case ROT_THETA_PLUS_PI:
        m(0, 0) = cos(PI + angle);
        m(0, 1) = -sin(PI + angle);
        m(1, 0) = sin(PI + angle);
        m(1, 1) = cos(PI + angle);
        break;

    default:
        cout << "Unknown RelationType" << endl;
        break;
    }

    return m;
}

Vector2d transformVector(Vector2d v, RelationType type)
{
    switch (type)
    {
    case IDENTITY:
        return v;

    case INVERTED:
        return -v;

    case ROT90:
        return Vector2d(-v[1], v[0]);

    case ROT270:
        return Vector2d(v[1], -v[0]);

    default:
        cout << "Unknown RelationType" << endl;
        break;
    }
}

void savePoints(std::string fname, vector<Point> &pts)
{
    ofstream file(fname);

    for (auto &p : pts)
    {
        file << p[0] << " " << p[1] << "\n";
    }

    file.close();
}

namespace
{
    Eigen::Matrix<double, 2, -1> concatTransforms(std::vector<Eigen::Matrix2d> &trafos)
    {
        Eigen::Matrix<double, 2, -1> ret(2, 2 * trafos.size());

        for (int i = 0; i < trafos.size(); ++i)
            ret.block(0, 2 * i, 2, 2) = trafos[i];

        return ret;
    }
}

vector<Eigen::Matrix<double, 2, -1> >
MetaGrid::propagateEdgeDOFs(const vector<int> &dofs)
{
    vector<vector<Eigen::Matrix2d> > ret(edges.size(), vector<Eigen::Matrix2d>(dofs.size(), Eigen::Matrix2d::Zero()));

    const int n = edgeGraph.size();
    vector<char> visited(n, false);

    int cnt = 0;
    for (int i : dofs)
    {
        ret[i][cnt] = Eigen::Matrix2d::Identity();
        vector<int> comp{i};
        visited[i] = true;

        int pos = 0;
        while (pos < comp.size())
        {
            int k = comp[pos++];

            for (auto i : edgeGraph[k])
            {
                if (i.i != -1 && !visited[i.i])
                {
                    ret[i.i][cnt] = transformation(i.type) * ret[k][cnt];

                    comp.push_back(i.i);
                    visited[i.i] = true;
                }
            }
        }

        ++cnt;
    }

    vector<Eigen::Matrix<double, 2, -1> > ret2(edges.size());

    for (int i = 0; i < edges.size(); ++i)
        ret2[i] = concatTransforms(ret[i]);

    return ret2;
}

vector<Eigen::Matrix<double, 2, -1> >
MetaGrid::propagateEdgeDOFsActive(const vector<int> &dofs, vector<double> angles)
{
    vector<vector<Eigen::Matrix2d> > ret(edges.size(), vector<Eigen::Matrix2d>(dofs.size(), Eigen::Matrix2d::Zero()));

    const int n = edgeGraph.size();
    vector<char> visited(n, false);

    int cnt = 0;
    for (int i : dofs)
    {
        ret[i][cnt] = Eigen::Matrix2d::Identity();
        vector<int> comp{i};
        visited[i] = true;

        int pos = 0;
        while (pos < comp.size())
        {
            int k = comp[pos++];

            for (auto i : edgeGraph[k])
            {
                if (i.i != -1 && !visited[i.i])
                {
                    ret[i.i][cnt] = transformationActive(i.type, angles[i.cell_index]) * ret[k][cnt];

                    comp.push_back(i.i);
                    visited[i.i] = true;
                }
            }
        }

        ++cnt;
    }

    vector<Eigen::Matrix<double, 2, -1> > ret2(edges.size());

    for (int i = 0; i < edges.size(); ++i)
        ret2[i] = concatTransforms(ret[i]);

    return ret2;
}

vector<Eigen::Matrix2d> add(const vector<Eigen::Matrix2d> &A, const vector<Eigen::Matrix2d> &B)
{
    const int n = A.size();
    vector<Eigen::Matrix2d> ret(n, Eigen::Matrix2d::Zero());

    for (int i = 0; i < n; ++i)
        ret[i] = A[i] + B[i];

    return ret;
}

vector<Eigen::Matrix2d> invert(vector<Eigen::Matrix2d> A)
{
    for (auto &a : A)
        a *= -1.0;
    return A;
}

vector<Eigen::Matrix<double, 2, -1> >
MetaGrid::propagateVertexDOFs(const vector<Eigen::Matrix<double, 2, -1> > &edgeTransformations)
{
    const int ndofs = edgeTransformations.front().cols();

    vector<Eigen::Matrix<double, 2, -1> > ret(vertices.size());
    for (auto &x : ret)
        x.setZero();

    vector<vector<pair<int, Eigen::Matrix<double, 2, -1> > > > gridGraph(vertices.size());

    int cnt = 0;
    for (auto &e : edges)
    {
        gridGraph[e.i].push_back(make_pair(e.j, edgeTransformations[cnt]));
        gridGraph[e.j].push_back(make_pair(e.i, -edgeTransformations[cnt]));

        ++cnt;
    }

    int anchor = anchors.front();
    vector<char> flag(vertices.size(), false);
    vector<int> path{anchor};
    vector<Eigen::Matrix<double, 2, -1> > pathT(1, Eigen::Matrix<double, 2, -1>(2, ndofs));
    pathT.front().setZero();

    while (1)
    {
        int id = path.back();
        bool f = false;

        for (auto &e : gridGraph[id])
        {
            if (!flag[e.first])
            {
                f = true;
                path.push_back(e.first);
                pathT.push_back(e.second + pathT.back());

                ret[e.first] = pathT.back();
                flag[e.first] = true;
                break;
            }
        }

        if (!f)
        {
            path.pop_back();
            pathT.pop_back();
            if (path.empty())
                break;
        }
    }

    return ret;
}

vector<Point>
MetaGrid::propagateDOFs(const vector<pair<int, Vector2d> > &dofs)
{
    const int n = edgeGraph.size();
    const int ne = edges.size();
    const int ndof = dofs.size();

    vector<int> dofIds;
    for (auto &x : dofs)
        dofIds.push_back(x.first);

    auto edgeTransformations = propagateEdgeDOFs(dofIds);
    vector<Vector2d> eVectors(ne, Vector2d(0, 0));

    for (int i = 0; i < ne; ++i)
    {
        for (int j = 0; j < ndof; ++j)
            eVectors[i] += edgeTransformations[i].block(0, 2 * j, 2, 2) * dofs[j].second;
    }

    // integrate edge vectors
    vector<Point> points = vertices;
    vector<char> visited(vertices.size(), false);
    visited[anchors.front()] = true;

    while (1)
    {
        bool found = false;

        auto it = eVectors.begin();
        for (auto &e : edges)
        {
            if (visited[e.i] && !visited[e.j])
            {
                found = true;
                points[e.j] = points[e.i] + *it;
                visited[e.j] = true;
            }
            else if (visited[e.j] && !visited[e.i])
            {
                found = true;
                points[e.i] = points[e.j] - *it;
                visited[e.i] = true;
            }

            ++it;
        }

        if (!found)
            break;
    }

    return points;
}

vector<Point>
MetaGrid::propagateDOFsActive(const vector<pair<int, Vector2d> > &dofs, vector<double> angles)
{
    const int n = edgeGraph.size();
    const int ne = edges.size();
    const int ndof = dofs.size();

    vector<int> dofIds;
    for (auto &x : dofs)
        dofIds.push_back(x.first);

    auto edgeTransformations = propagateEdgeDOFsActive(dofIds, angles);
    vector<Vector2d> eVectors(ne, Vector2d(0, 0));

    for (int i = 0; i < ne; ++i)
    {
        for (int j = 0; j < ndof; ++j)
            eVectors[i] += edgeTransformations[i].block(0, 2 * j, 2, 2) * dofs[j].second;
    }

    // integrate edge vectors
    vector<Point> points = vertices;
    vector<char> visited(vertices.size(), false);
    visited[anchors.front()] = true;

    while (1)
    {
        bool found = false;

        auto it = eVectors.begin();
        for (auto &e : edges)
        {
            if (visited[e.i] && !visited[e.j])
            {
                found = true;
                points[e.j] = points[e.i] + *it;
                visited[e.j] = true;
            }
            else if (visited[e.j] && !visited[e.i])
            {
                found = true;
                points[e.i] = points[e.j] - *it;
                visited[e.i] = true;
            }

            ++it;
        }

        if (!found)
            break;
    }

    return points;
}

vector<Point>
MetaGrid::setDOFs(const vector<int> &dofs, const vector<double> &values)
{
    //cout << dofs.size() << " " << values.size() << endl;
    assert(dofs.size() * 2 == values.size());
    vector<pair<int, Vector2d> > dofValues;

    auto it = values.begin();
    for (int i : dofs)
    {
        double x = *it++;
        double y = *it++;
        dofValues.push_back(make_pair(i, Vector2d(x, y)));
    }

    return propagateDOFs(dofValues);
}

vector<Point>
MetaGrid::setDOFsActive(const vector<int> &dofs, const vector<double> &values, vector<double> angles)
{
    //cout << dofs.size() << " " << values.size() << endl;
    assert(dofs.size() * 2 == values.size());
    vector<pair<int, Vector2d> > dofValues;

    auto it = values.begin();
    for (int i : dofs)
    {
        double x = *it++;
        double y = *it++;
        dofValues.push_back(make_pair(i, Vector2d(x, y)));
    }

    return propagateDOFsActive(dofValues, angles);
}

void MetaGrid::setEdges()
{
    for (auto &c : cells)
    {
        for (int k = 0; k < 4; ++k)
        {
            int a = c[k];
            int b = c[(k + 1) % 4];

            auto e = Edge(min(a, b), max(a, b));
            auto it = find(edges.begin(), edges.end(), e);

            if (it == edges.end())
            {
                edges.push_back(e);
                c.edges[k].first = edges.size() - 1;
            }
            else
                c.edges[k].first = distance(edges.begin(), it);

            c.edges[k].second = a < b ? IDENTITY : INVERTED;
        }
    }
}

void MetaGrid::setEdgeRelations()
{
    edgeGraph.clear();
    edgeGraph.resize(edges.size());

    auto addEdge = [&](const int ind, const int i, const int j, RelationType type)
    {
        RelationType type2 = type;
        if (type == ROT270)
            type2 = ROT90;
        if (type == ROT90)
            type2 = ROT270;
        if (type == ROT_PI_MINUS_THETA)
            type2 = ROT_THETA_PLUS_PI;
        if (type == ROT_2PI_MINUS_THETA)
            type2 = ROT_THETA;
        if (type == ROT_THETA)
            type2 = ROT_2PI_MINUS_THETA;
        if (type == ROT_THETA_PLUS_PI)
            type2 = ROT_PI_MINUS_THETA;

        edgeGraph[i].push_back(RelEdge(j, type, ind));
        edgeGraph[j].push_back(RelEdge(i, type2, ind));
    };


    int cell_index = 0;
    for (auto &c : cells)
    {
        if (c.type == SHEAR)
        {
            addEdge(cell_index, c.edges[0].first, c.edges[2].first, c.edges[0].second == c.edges[2].second ? INVERTED : IDENTITY);
            addEdge(cell_index, c.edges[1].first, c.edges[3].first, c.edges[1].second == c.edges[3].second ? INVERTED : IDENTITY);
        }
        else if (c.type == RIGID)
        {
            addEdge(cell_index, c.edges[0].first, c.edges[1].first, c.edges[0].second == c.edges[1].second ? ROT90 : ROT270);
            addEdge(cell_index, c.edges[0].first, c.edges[2].first, c.edges[0].second == c.edges[2].second ? INVERTED : IDENTITY);
            addEdge(cell_index, c.edges[0].first, c.edges[3].first, c.edges[0].second == c.edges[3].second ? ROT270 : ROT90);

            addEdge(cell_index, c.edges[1].first, c.edges[2].first, c.edges[1].second == c.edges[2].second ? ROT90 : ROT270);
            addEdge(cell_index, c.edges[1].first, c.edges[3].first, c.edges[1].second == c.edges[3].second ? INVERTED : IDENTITY);

            addEdge(cell_index, c.edges[2].first, c.edges[3].first, c.edges[2].second == c.edges[3].second ? ROT90 : ROT270);
        }
        else if (c.type == ACTIVE)
        {
            addEdge(cell_index, c.edges[0].first, c.edges[1].first, c.edges[0].second == c.edges[1].second ? ROT_THETA : ROT_THETA_PLUS_PI);
            addEdge(cell_index, c.edges[0].first, c.edges[2].first, c.edges[0].second == c.edges[2].second ? INVERTED : IDENTITY);
            addEdge(cell_index, c.edges[0].first, c.edges[3].first, c.edges[0].second == c.edges[3].second ? ROT_THETA_PLUS_PI : ROT_THETA);

            addEdge(cell_index, c.edges[1].first, c.edges[2].first, c.edges[1].second == c.edges[2].second ? ROT_PI_MINUS_THETA : ROT_2PI_MINUS_THETA);
            addEdge(cell_index, c.edges[1].first, c.edges[3].first, c.edges[1].second == c.edges[3].second ? INVERTED : IDENTITY);

            addEdge(cell_index, c.edges[2].first, c.edges[3].first, c.edges[2].second == c.edges[3].second ? ROT_THETA : ROT_THETA_PLUS_PI);
        }
        cell_index++;
    }
}

vector<int>
MetaGrid::degreesOfFreedom()
{
    auto cc = connectedComponents(edgeGraph);
    vector<int> dofs;

    for (auto &c : cc)
        dofs.push_back(c.front());
    return dofs;
}

MetaGrid::MetaGrid(const GridModel &model)
{
    vertices = model.points;

    // copy cell information
    for (auto &cell : model.cells)
        cells.emplace_back(cell.type,
                           cell.vertices[0],
                           cell.vertices[1],
                           cell.vertices[2],
                           cell.vertices[3]);

    // constraints
    const int numInputs = model.inputs.size();
    assert(model.inputPaths.size() == numInputs);
    // std::cout << numInputs << std::endl;

    for (int i = 0; i < numInputs; ++i)
        constrained.push_back(make_pair(model.inputs[i],
                                        model.inputPaths[i]));

    const int numTargets = model.targets.size();
    assert(model.targetPaths.size() == numTargets);
    // std::cout << numTargets << std::endl;

    for (int i = 0; i < numTargets; ++i)
        constrained.push_back(make_pair(model.targets[i],
                                        model.targetPaths[i]));

    // anchors
    anchors = model.anchors;
    assert(!anchors.empty());
    shift = vertices[model.anchors.front()];

    for (auto &v : vertices)
        v -= shift;

    for (auto &c : constrained)
        for (auto &p : c.second)
            p -= shift;

    for (int i : anchors)
        anchorPositions.push_back(vertices[i]);
}
