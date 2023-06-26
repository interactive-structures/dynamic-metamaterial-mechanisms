#define _CRT_SECURE_NO_WARNINGS

#include <igl/opengl/glfw/Viewer.h>

using namespace Eigen;
std::pair<MatrixX3d, MatrixX3i> generateCapsule(Vector3d base, double r, double h, int res, double rot);
std::pair<MatrixX3d, MatrixX3i> combineMeshes(std::vector<std::pair<MatrixX3d, MatrixX3i>> meshes);