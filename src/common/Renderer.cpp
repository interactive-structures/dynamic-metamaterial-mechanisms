#include "Renderer.hpp"

using namespace Eigen;

Renderer::Renderer(int resolution) {
    this->resolution = resolution;
    debug_points = {};
    debug_edges = {};
    debug_pointColors = {};
    debug_edgeColors = {};
    face_points = {};
    face_triangles = {};
    point_map = {};
}
void Renderer::clear() {
    this->resolution = resolution;
    debug_points = {};
    debug_edges = {};
    debug_pointColors = {};
    debug_edgeColors = {};
    face_points = {};
    face_triangles = {};
    point_map = {};
}
void Renderer::renderTo(Viewer &v) {
    v.data().clear();
    v.data().set_points(debug_points, debug_pointColors);
    v.data().set_edges(debug_points, debug_edges, debug_edgeColors);
    v.data().set_mesh(face_points, face_triangles);
}

void Renderer::addSphere(RenderTag tag, Position center, double radius) {
    std::cout << tag << std::endl;
    int oldNumPoints = face_points.rows();
    int oldNumFaces = face_triangles.rows();
    int numPoints = resolution * resolution;
    int numFaces = (resolution - 1) * resolution * 2;
    face_points.conservativeResize(oldNumPoints + numPoints, 3);
    face_triangles.conservativeResize(oldNumFaces + numFaces, 3);
    // std::cout << oldNumPoints << " -points-> " << face_points.rows() << std::endl;
    // std::cout << oldNumFaces << " -triangles-> " << face_triangles.rows() << std::endl << std::endl;

    Vector3d centerVec(center[0], center[1], center[2]);
    double half_res = (double) (resolution - 1) / 2.0;
    double angle_step = 2.0 * M_PI / (double)resolution;
    int pointIndex = oldNumPoints;
    for(int height_index = 0; height_index < resolution; height_index++) {
        double offset_hi = height_index - half_res;
        double ratio = offset_hi / half_res;
        std::cout << "Ratio: " << ratio << std::endl;
        double radius_for_height = sqrt(1 - (ratio * ratio)) * radius;
        for(int circ_index = 0; circ_index < resolution; circ_index ++) {
            double angle = circ_index * angle_step;
            face_points.row(pointIndex) = centerVec + Vector3d(radius_for_height * cos(angle), radius_for_height * sin(angle), ratio * radius);
            pointIndex++;
        }
    }
    int faceIndex = oldNumFaces;
    Vector3i pointIndexOffset(oldNumPoints, oldNumPoints, oldNumPoints);
    for(int i = 0; i < resolution - 1; i++) {
        int indexOffset = i * resolution;
        for(int j = 0; j < resolution; j++) {
            int corner1 = j + indexOffset;
            int corner2 = (j + 1) % resolution + indexOffset;
            int corner3 = (j + 1) % resolution + indexOffset + resolution;
            int corner4 = j + indexOffset + resolution;
            face_triangles.row(faceIndex) = Vector3i(corner1, corner2, corner3) + pointIndexOffset;
            faceIndex++;
            face_triangles.row(faceIndex) = Vector3i(corner3, corner4, corner1) + pointIndexOffset;
            faceIndex++;
        }
    }
}

void Renderer::addCylinder(RenderTag tag,  Position posA, Position posB, double radius) {
    // Compute the direction vector from pointA to pointB
    std::cout << tag << std::endl;
    Vector3d pointA(posA[0], posA[1], posA[2]), pointB(posB[0], posB[1], posB[2]);
    Eigen::Vector3d direction = pointB - pointA;
    double length = direction.norm();
    direction.normalize();
    Vector3d orth(copysign(direction.z(), direction.x()), copysign(direction.z(), direction.y()), -copysign(direction.x(), direction.z()) - copysign(direction.y(), direction.z())); //from: https://math.stackexchange.com/q/4112622
    orth.normalize();

    int oldNumPoints = face_points.rows();
    int oldNumFaces = face_triangles.rows();
    int numPoints = 2* resolution;
    int numFaces = 2 * resolution + 2 * (resolution - 2);
    face_points.conservativeResize(oldNumPoints + numPoints, 3);
    face_triangles.conservativeResize(oldNumFaces + numFaces, 3);
    // std::cout << oldNumPoints << " -points-> " << face_points.rows() << std::endl;
    // std::cout << oldNumFaces << " -triangles-> " << face_triangles.rows() << std::endl << std::endl;

    // Generate points for the cylinder
    int pointIndex = oldNumPoints;
    for (int i = 0; i < resolution; ++i) {
        double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(resolution);
        face_points.row(pointIndex) = pointA + radius * (AngleAxisd(angle, direction) * orth);
        pointIndex++;
    }
    for (int i = 0; i < resolution; ++i) {
        double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(resolution);
        face_points.row(pointIndex) = pointA + radius * (AngleAxisd(angle, direction) * orth) + length * direction;
        pointIndex++;
    }

    // Generate faces for the cylinder body
    int faceIndex = oldNumFaces;
    Vector3i pointIndexOffset(oldNumPoints, oldNumPoints, oldNumPoints);
    for (int i = 0; i < resolution; ++i) {
        int corner1 = i;
        int corner2 = (i + 1) % resolution;
        int corner3 = (i + 1) % resolution + resolution;
        int corner4 = i + resolution;
        face_triangles.row(faceIndex) = Vector3i(corner1, corner2, corner3) + pointIndexOffset;
        faceIndex++;
        face_triangles.row(faceIndex) = Vector3i(corner3, corner4, corner1) + pointIndexOffset;
        faceIndex++;
    }
    //Generate faces for cylinder ends
    for (int i = 1; i < resolution - 1; ++i) {
        int corner1 = 0;
        int corner2 = i;
        int corner3 = (i + 1) % resolution;
        face_triangles.row(faceIndex) = Vector3i(corner3, corner2, corner1) + pointIndexOffset;
        faceIndex++;
        face_triangles.row(faceIndex) = Vector3i(corner1 + resolution, corner2 + resolution, corner3 + resolution) + pointIndexOffset;
        faceIndex++;
    }
}

void Renderer::addCapsule(RenderTag tag,  Position posA, Position posB, double radius) {
    addSphere(tag + "__SphereA", posA, radius);
    addSphere(tag + "__SphereB", posB, radius);
    addCylinder(tag + "__Cylinder", posA, posB, radius);
}

void Renderer::addCell(RenderTag tag, vector<Position> corners, double width, double thickness) {
    int len = corners.size();
    int oldNumPoints = face_points.rows();
    int oldNumFaces = face_triangles.rows();
    int numPoints = 4 * len;
    int numFaces = 8 * len;
    face_points.conservativeResize(oldNumPoints + numPoints, 3);
    face_triangles.conservativeResize(oldNumFaces + numFaces, 3);
    // std::cout << oldNumPoints << " -points-> " << face_points.rows() << std::endl;
    // std::cout << oldNumFaces << " -triangles-> " << face_triangles.rows() << std::endl << std::endl;

    int pointIndex = oldNumPoints;
    Vector3d center(0,0,0);
    for(Position pos : corners) {
        center += Vector3d(pos[0], pos[1], 0);
    }
    center /= len;
    for(Position pos : corners) {
        Vector3d direction = center - Vector3d(pos[0], pos[1], 0);
        direction.normalize();
        face_points.row(pointIndex) = Vector3d(pos[0], pos[1], -width / 2.0);
        face_points.row(pointIndex + len) = Vector3d(pos[0], pos[1], width/2.0);
        face_points.row(pointIndex + 2 * len) = Vector3d(pos[0], pos[1], -width / 2.0) + thickness * direction;
        face_points.row(pointIndex + 3 * len) = Vector3d(pos[0], pos[1], width/2.0) + thickness * direction;
        pointIndex++;
    }

    int faceIndex = oldNumFaces;
    Vector3i pointIndexOffset(oldNumPoints, oldNumPoints, oldNumPoints);
    for(int i = 0; i < len; i++) {
        int outer_corner1 = i;
        int outer_corner2 = (i + 1) % len;
        int outer_corner3 = (i + 1) % len + len;
        int outer_corner4 = i + len;
        int inner_offset = 2 * len;
        int corner1 = i + inner_offset;
        int corner2 = (i + 1) % len + inner_offset;
        int corner3 = (i + 1) % len + len + inner_offset;
        int corner4 = i + len + inner_offset;
        face_triangles.row(faceIndex) = Vector3i(corner1, corner2, corner3) + pointIndexOffset;
        faceIndex++;
        face_triangles.row(faceIndex) = Vector3i(corner3, corner4, corner1) + pointIndexOffset;
        faceIndex++;
        face_triangles.row(faceIndex) = Vector3i(outer_corner3, outer_corner2, outer_corner1) + pointIndexOffset;
        faceIndex++;
        face_triangles.row(faceIndex) = Vector3i(outer_corner1, outer_corner4, outer_corner3) + pointIndexOffset;
        faceIndex++;
        face_triangles.row(faceIndex) = Vector3i(corner1, corner2, outer_corner2) + pointIndexOffset;
        faceIndex++;
        face_triangles.row(faceIndex) = Vector3i(outer_corner2, outer_corner1, corner1) + pointIndexOffset;
        faceIndex++;
        face_triangles.row(faceIndex) = Vector3i(outer_corner3, corner3, corner4) + pointIndexOffset;
        faceIndex++;
        face_triangles.row(faceIndex) = Vector3i(corner4, outer_corner4, outer_corner3) + pointIndexOffset;
        faceIndex++;
    }
}