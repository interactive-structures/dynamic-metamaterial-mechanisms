#include <unordered_map>
#include <vector>
#include <igl/opengl/glfw/Viewer.h>
#include "Position.hpp"

typedef std::string RenderTag;

using std::vector;
using std::unordered_map;
using igl::opengl::glfw::Viewer;
using namespace Eigen;

class Renderer {
    private:
        int resolution;
        MatrixX3d debug_points;
        MatrixX3d debug_pointColors;
        MatrixX2i debug_edges;
        MatrixX3d debug_edgeColors;
        MatrixX3d face_points;
        MatrixX3i face_triangles;
        unordered_map<RenderTag, vector<int>> point_map;
    public:
        Renderer(int resolution);
        void renderTo(Viewer &v);
        void clear();
        void addSphere(RenderTag tag, Position center, double radius);
        void addCylinder(RenderTag tag,  Position posA, Position posB, double radius);
        void addCapsule(RenderTag tag, Position posA, Position posB, double radius);
        void addCell(RenderTag tag, vector<Position> corners, double width, double thickness);
        void addDebugPoint(Position pos);
        void updateSphere(RenderTag tag, Position center, double radius);
        void updateCylinder(RenderTag tag,  Position center, double radius, double halfHeight);
        void updateBevelSegment(RenderTag tag, Position center, double radius, double halfHeight);
        void updateCell(RenderTag tag, vector<Position> corners);
};