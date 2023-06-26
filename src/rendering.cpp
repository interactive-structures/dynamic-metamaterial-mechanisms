#include "rendering.hpp"

// mesh generation test
using namespace Eigen;
std::pair<MatrixX3d, MatrixX3i> generateCapsule(Vector3d base, double r, double h, int res, double rot)
{
    MatrixX3d V;
    MatrixX3i F;
    const double pi = M_PI;
    double rad_amount = 2 * pi / res;

    Vector3d axis(0, 0, 1);
    Transform<double, 3, Affine> t(AngleAxis(rot, axis));
    // std::cout << rad_amount << std::endl;
    std::vector<Vector3d> points;
    for (int i = 0; i < res; i++)
    {
        double angle1 = i * rad_amount;
        double c1 = cos(angle1), s1 = sin(angle1);
        for (int j = 0; j < res / 2; j++)
        {
            double angle2 = j * rad_amount / 2;
            double littleR = cos(angle2) * r;
            double littleH = sin(angle2) * r;
            double x = littleR * c1;
            double z = littleR * s1;
            Vector3d o1, o2;
            o1 << x, -littleH, z;
            o2 << x, h + littleH, z;
            points.push_back(base + t * o1);
            points.push_back(base + t * o2);
        }
    }
    std::vector<Vector3i> faces;
    int index_counter = 0;
    for (int i = 0; i < res; i++)
    {
        int next_i = index_counter + 1;
        int across_i = (index_counter + res) % points.size();
        int across_next_i = (next_i + res) % points.size();
        Vector3i f1, f2;
        f1 << index_counter, next_i, across_next_i;
        f2 << across_next_i, across_i, index_counter;
        faces.push_back(f1);
        faces.push_back(f2);
        // std::cout << "Connecting " << f1 << " and also " << f2 << std::endl;
        for (int j = 0; j < res - 2; j++)
        {
            int next_i = index_counter + 2;
            int across_i = (index_counter + res) % points.size();
            int across_next_i = (next_i + res) % points.size();
            Vector3i f1, f2;

            if(index_counter % 2 == 0) {
                f1 << across_next_i, next_i, index_counter;
                f2 << index_counter, across_i, across_next_i;
            }
            else {
                f1 << index_counter, next_i, across_next_i;
                f2 << across_next_i, across_i, index_counter;
            }

            // std::cout << "Connecting " << f1 << " and also " << f2 << std::endl;
            faces.push_back(f1);
            faces.push_back(f2);
            index_counter++;
        }
        index_counter += 2;
    }

    V = MatrixX3d::Zero(points.size(), 3);
    for (int i = 0; i < points.size(); i++)
    {
        V.row(i) = points[i];
    }
    F = MatrixX3i::Zero(faces.size(), 3);
    for (int i = 0; i < faces.size(); i++)
    {
        F.row(i) = faces[i];
    }
    return std::make_pair(V, F);
}

std::pair<MatrixX3d, MatrixX3i> combineMeshes(std::vector<std::pair<MatrixX3d, MatrixX3i>> meshes)
{
    int vrows = 0;
    int frows = 0;
    for(auto mesh : meshes) {
        vrows += mesh.first.rows();
        frows += mesh.second.rows();
    }
    MatrixX3d V(vrows, 3);
    MatrixX3i F(frows, 3);
    int v_offset = 0;
    int f_offset = 0;
    for(auto mesh : meshes) {
        auto v = mesh.first;
        auto f = mesh.second;
        V.middleRows(v_offset, v.rows()) = v;
        MatrixX3i indexOffset = MatrixX3i::Constant(f.rows(), 3, v_offset);
        MatrixX3i new_faces = f + indexOffset;
        F.middleRows(f_offset, f.rows()) = new_faces;
        v_offset += v.rows();
        f_offset += f.rows();
    }
    return std::make_pair(V, F);
}