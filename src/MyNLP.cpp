#include "MyNLP.hpp"

#include <cassert>
#include <array>
#include <iostream>

using namespace std;
using namespace Ipopt;

Eigen::Matrix<double, 2, -1> concatTransforms(std::vector<Eigen::Matrix2d> &trafos)
{
    Eigen::Matrix<double, 2, -1> ret(2, 2 * trafos.size());

    for (int i = 0; i < trafos.size(); ++i)
        ret.block(0, 2 * i, 2, 2) = trafos[i];

    return ret;
}

Eigen::Matrix<double, 2, -1> rotate(Eigen::Matrix<double, 2, -1> m)
{
    Eigen::Matrix<double, 2, -1> ret = m;

    Eigen::Matrix2d rot2d;
    rot2d << 0, -1, 1, 0;

    for (int i = 0; i < ret.cols() / 2; ++i)
    {
        ret.block(0, 2 * i, 2, 2) = rot2d * ret.block(0, 2 * i, 2, 2);
    }

    return ret;
}

OrientationConstraint::OrientationConstraint(MetaGrid &grid, const Edge e)
{
    // find a cell containing the edge
    for (auto &c : grid.cells)
    {
        int cnt = 0;

        for (int k = 0; k < 4; ++k)
        {
            if (c[k] == e.i)
                ++cnt;
            if (c[k] == e.j)
                ++cnt;
        }

        if (cnt == 2)
        {
            A0 = rotate(grid.vertexTransformations[c[1]] - grid.vertexTransformations[c[0]]);
            A1 = grid.vertexTransformations[c[3]] - grid.vertexTransformations[c[0]];

            A = A0.transpose() * A1 + A1.transpose() * A0;

            break;
        }
    }
}

OrientationConstraint::OrientationConstraint(MetaGrid &grid, const Cell c)
{
    A0 = rotate(grid.vertexTransformations[c[1]] - grid.vertexTransformations[c[0]]);
    A1 = grid.vertexTransformations[c[3]] - grid.vertexTransformations[c[0]];

    A = A0.transpose() * A1 + A1.transpose() * A0;
}

double OrientationConstraint::value(const double *x, const int len)
{
    Eigen::MatrixXd Ax = A0.transpose() * A1;
    return Eigen::Map<const Eigen::VectorXd>(x, len).transpose() * Ax * Eigen::Map<const Eigen::VectorXd>(x, len);
}

Eigen::VectorXd OrientationConstraint::grad(const double *x, const int len)
{
    return A * Eigen::Map<const Eigen::VectorXd>(x, len);
}

Eigen::MatrixXd OrientationConstraint::hess()
{
    return A;
}

MyNLP::MyNLP(MetaGrid &_grid)
    : grid(_grid), start(_grid.vertices)
{
    dofs = grid.degreesOfFreedom();

    auto edgeTraf = grid.propagateEdgeDOFs(dofs);
    vertexTransformations = grid.propagateVertexDOFs(edgeTraf);
    grid.vertexTransformations = vertexTransformations;

    for (auto &c : grid.cells)
    {
        if (c.type == SHEAR)
            orientationConstraints.emplace_back(grid, c);
    }

    /*
    for(int d : dofs)
    {
        orientationConstraints.emplace_back(grid, grid.edges[d]);
    }*/
}

MyNLP::MyNLP(MetaGrid &_grid, std::vector<double> angles)
    : grid(_grid), start(_grid.vertices)
{
    dofs = grid.degreesOfFreedom();

    auto edgeTraf = grid.propagateEdgeDOFsActive(dofs, angles);
    vertexTransformations = grid.propagateVertexDOFs(edgeTraf);
    grid.vertexTransformations = vertexTransformations;

    for (auto &c : grid.cells)
    {
        if (c.type == SHEAR)
            orientationConstraints.emplace_back(grid, c);
    }

    /*
    for(int d : dofs)
    {
        orientationConstraints.emplace_back(grid, grid.edges[d]);
    }*/
}

MyNLP::~MyNLP()
{
}

void MyNLP::setConstraint(const int vid, Eigen::Vector2d pos)
{
    constrId.push_back(vid);
    constrTarget.push_back(pos);
}

bool MyNLP::get_nlp_info(Index &n, Index &m, Index &nnz_jac_g,
                         Index &nnz_h_lag, IndexStyleEnum &index_style)
{
    const int nd = dofs.size();
    const int na = grid.anchors.size() - 1; // first anchor is fixed implicitly

    const int no = orientationConstraints.size();

    n = 2 * nd;
    m = nd + no + na;

    nnz_jac_g = nd * 2 + no * 2 * nd + na * 2 * nd;

    nnz_h_lag = n * (n + 1) / 2; // hessian is dense

    index_style = C_STYLE;

    return true;
}

bool MyNLP::get_bounds_info(Index n, Number *x_l, Number *x_u,
                            Index m, Number *g_l, Number *g_u)
{
    // no bounds on variables
    const int nd = dofs.size();
    auto it = fixedDofs.begin();

    if (x_l && x_u)
    {
        for (int i = 0; i < nd; ++i)
        {
            if (it != fixedDofs.end() && i == *it)
            {
                // original edge coordinates
                auto e = grid.edges[dofs[i]];
                Vector2d edg = grid.vertices[e.j] - grid.vertices[e.i];

                x_l[2 * i] = edg[0];
                x_u[2 * i] = edg[0];

                x_l[2 * i + 1] = edg[1];
                x_u[2 * i + 1] = edg[1];

                ++it;
            }
            else
            {
                x_l[2 * i] = -1.0e19;
                x_u[2 * i] = 1.0e19;

                x_l[2 * i + 1] = -1.0e19;
                x_u[2 * i + 1] = 1.0e19;
            }
        }
    }

    /*
    for(int i = 0; i < n; ++i)
    {
        x_l[i] = -1.0e19;
        x_u[i] = 1.0e19;
    }   
    */

    for (int i = 0; i < n / 2; ++i)
    {
        g_l[i] = 1.0;
        g_u[i] = 1.0;
    }

    int offset = n / 2;

    for (int i = 0; i < orientationConstraints.size(); ++i)
    {
        g_l[offset + i] = sqrt(3) / 2;
        g_u[offset + i] = 1.0e19;
    }

    offset += orientationConstraints.size();

    for (int i = 0; i < (grid.anchors.size() - 1); ++i)
    {
        g_l[offset + i] = 0.0;
        g_u[offset + i] = 0.0;
    }

    return true;
}

void MyNLP::setFixedDof(const int dof)
{
    fixedDofs.push_back(dof);
}

void MyNLP::setStartConfiguration(const std::vector<Point> &init)
{
    start = init;
}

bool MyNLP::get_starting_point(Index n, bool init_x, Number *x,
                               bool init_z, Number *z_L, Number *z_U,
                               Index m, bool init_lambda,
                               Number *lambda)
{
    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);

    int cnt = 0;
    for (auto i : dofs)
    {
        Vector2d edge = start[grid.edges[i].j] - start[grid.edges[i].i];

        x[cnt] = edge[0];
        x[cnt + 1] = edge[1];

        cnt += 2;
    }

    return true;
}

bool MyNLP::eval_f(Index n, const Number *x, bool new_x, Number &obj_value)
{
    obj_value = 0.0;
    int N = constrId.size();

    for (int i = 0; i < N; ++i)
    {
        const double nrm = (vertexTransformations[constrId[i]] * Eigen::Map<const Eigen::VectorXd>(x, n) - constrTarget[i]).norm();
        obj_value += nrm * nrm;
    }

    const int nc = grid.cells.size();

    for (int i = 0; i < nc; ++i)
    {
    }

    return true;
}

bool MyNLP::eval_grad_f(Index n, const Number *x, bool new_x, Number *grad_f)
{

    for (int i = 0; i < n; ++i)
        grad_f[i] = 0.0;

    int N = constrId.size();

    for (int i = 0; i < N; ++i)
    {
        auto A = vertexTransformations[constrId[i]];

        Eigen::VectorXd grad = 2 * A.transpose() * A * Eigen::Map<const Eigen::VectorXd>(x, n) - 2. * A.transpose() * constrTarget[i];

        for (int i = 0; i < n; ++i)
            grad_f[i] += grad(i);
    }

    return true;
}

bool MyNLP::eval_g(Index n, const Number *x, bool new_x, Index m, Number *g)
{
    const int ndofs = dofs.size();

    auto A0 = rotate(vertexTransformations[1]);
    auto A1 = vertexTransformations[3];

    Eigen::MatrixXd A = A0.transpose() * A1;
    Eigen::VectorXd x2 = Eigen::Map<const Eigen::VectorXd>(x, n);

    // unit length constraint
    int i = 0;
    for (; i < ndofs; ++i)
    {
        g[i] = x[2 * i] * x[2 * i] + x[2 * i + 1] * x[2 * i + 1];
    }

    // non-flip constraint
    for (auto &c : orientationConstraints)
    {
        g[i++] = c.value(x, n);
    }

    // anchor constraits
    const int na = grid.anchors.size();

    for (int j = 1; j < na; ++j)
    {
        double ai = (vertexTransformations[grid.anchors[j]] * Eigen::Map<const Eigen::VectorXd>(x, n) - grid.anchorPositions[j]).norm();
        g[i++] = ai * ai;
    }

    assert(i == m);

    return true;
}

bool MyNLP::eval_jac_g(Index n, const Number *x, bool new_x,
                       Index m, Index nele_jac, Index *iRow, Index *jCol,
                       Number *values)
{
    const int ndofs = dofs.size();
    const int no = orientationConstraints.size();

    if (values == NULL)
    {

        int cnt = 0;

        // unit length constraint pattern
        for (int i = 0; i < ndofs; ++i)
        {
            iRow[cnt] = i;
            jCol[cnt] = 2 * i;

            ++cnt;

            iRow[cnt] = i;
            jCol[cnt] = 2 * i + 1;

            ++cnt;
        }

        int rowOffset = ndofs;

        // non-flip constraint
        for (int j = 0; j < no; ++j)
        {

            for (int k = 0; k < 2 * ndofs; ++k)
            {
                iRow[cnt] = j + rowOffset;
                jCol[cnt] = k;

                ++cnt;
            }
        }

        rowOffset += no;

        // anchor constraints
        const int na = grid.anchors.size() - 1;
        for (int j = 0; j < na; ++j)
        {
            for (int k = 0; k < 2 * ndofs; ++k)
            {
                iRow[cnt] = j + rowOffset;
                jCol[cnt] = k;
                ++cnt;
            }
        }
    }
    else
    {

        // unit length constraint
        for (int i = 0; i < 2 * ndofs; ++i)
        {
            values[i] = 2 * x[i];
        }

        // non-flip constraint
        int i = 2 * ndofs;
        for (auto &c : orientationConstraints)
        {
            Eigen::VectorXd grad = c.grad(x, n);
            copy_n(grad.data(), 2 * ndofs, &values[i]);
            i += 2 * ndofs;
        }

        // anchor constraints
        const int na = grid.anchors.size();
        for (int l = 1; l < na; ++l)
        {
            auto A = vertexTransformations[grid.anchors[l]];
            Eigen::VectorXd grad = 2 * A.transpose() * A * Eigen::Map<const Eigen::VectorXd>(x, n) - 2. * A.transpose() * grid.anchorPositions[l];

            for (int j = 0; j < 2 * ndofs; ++j)
            {
                values[i++] = grad(j);
            }
        }

        assert(i == ndofs * 2 + no * 2 * ndofs + (na - 1) * 2 * ndofs);
    }

    return true;
}

bool MyNLP::eval_h(Index n, const Number *x, bool new_x,
                   Number obj_factor, Index m, const Number *lambda,
                   bool new_lambda, Index nele_hess, Index *iRow,
                   Index *jCol, Number *values)
{
    const int ndofs = dofs.size();

    if (values == NULL)
    {

        int cnt = 0;
        for (int i = 0; i < 2 * ndofs; ++i)
            for (int j = 0; j <= i; ++j)
            {
                iRow[cnt] = i;
                jCol[cnt] = j;

                ++cnt;
            }
    }
    else
    {

        Eigen::MatrixXd Ho(dofs.size() * 2, dofs.size() * 2);
        Ho.setZero();

        for (int i : constrId)
        {
            auto B = vertexTransformations[i];
            Ho += 2 * B.transpose() * B;
        }

        Eigen::MatrixXd H = obj_factor * Ho;

        // unit length contraint
        for (int i = 0; i < ndofs; ++i)
        {
            Eigen::MatrixXd Hc(2 * ndofs, 2 * ndofs);
            Hc.setZero();

            Hc(2 * i, 2 * i) = 2;
            Hc(2 * i + 1, 2 * i + 1) = 2;

            H += lambda[i] * Hc;
        }

        int cnt = ndofs;

        // non-flip constraint
        for (auto &c : orientationConstraints)
        {
            H += lambda[cnt++] * c.hess();
        }

        // anchor constraint
        for (int i = 1; i < grid.anchors.size(); ++i)
        {
            auto B = vertexTransformations[grid.anchors[i]];
            H += lambda[cnt++] * 2 * B.transpose() * B;
        }

        cnt = 0;
        for (int i = 0; i < 2 * ndofs; ++i)
            for (int j = 0; j <= i; ++j)
                values[cnt++] = H(i, j);
    }

    return true;
}

void MyNLP::finalize_solution(SolverReturn status,
                              Index n, const Number *x, const Number *z_L, const Number *z_U,
                              Index m, const Number *g, const Number *lambda,
                              Number obj_value,
                              const IpoptData *ip_data,
                              IpoptCalculatedQuantities *ip_cq)
{
    solution.resize(n);
    copy_n(x, n, solution.data());

    eval_f(n, x, true, objError);

    // squared constrained violation
    vector<double> gl(m), gu(m), gv(m);

    get_bounds_info(n, 0, 0,
                    m, gl.data(), gu.data());

    eval_g(n, x, true, m, gv.data());

    constrError = .0;

    for (int i = 0; i < m; ++i)
    {
        if (g[i] > gu[i])
        {
            constrError += pow(g[i] - gu[i], 2);
        }
        else if (g[i] < gl[i])
        {
            constrError += pow(g[i] - gl[i], 2);
        }
    }

    // cout << "Constraint: " << constrError << endl;
    // cout << "Objective : " << objError << endl
    //      << endl;
}
