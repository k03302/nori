#pragma once
#include <nori/common.h>
#include <nori/bbox.h>
#include <nori/mesh.h>
#include <queue>

NORI_NAMESPACE_BEGIN

class Triangle
{
public:
    static const Triangle &getTriangle(int fIndex, Mesh &mesh)
    {
        if (currentTriangleIndex >= TRIANGLE_COUNT)
            throw NoriException("Triangle::getTriangle: exceeded maximum triangle count");
        Triangle &triangle = triangles[currentTriangleIndex++];

        Vector3f v0, v1, v2;
        mesh.getVertex(fIndex, v0, v1, v2);
        triangle.m_min = v0.cwiseMin(v1).cwiseMin(v2);
        triangle.m_max = v0.cwiseMax(v1).cwiseMax(v2);
        triangle.m_fIndex = fIndex;
        triangle.m_mesh = &mesh;
        return triangle;
    }

private:
    int m_fIndex;
    Mesh *m_mesh;
    Point3f m_min, m_max;

    bool isValid() const
    {
        return m_mesh != nullptr;
    }

    static const int TRIANGLE_COUNT = 10000000;
    static Triangle triangles[TRIANGLE_COUNT];
    static int currentTriangleIndex;
};

struct OctNodeContextedTriangle
{
    int m_octNodeOverlap;
    int m_overlapCount;
    Triangle *m_triangle;
};

class OctNode
{
public:
private:
    OctNodeContextedTriangle m_triangles[8];
    int m_maxOverlapCount;
};

template <int MaxDepth>
class Octree
{
public:
    void build(const std::vector<Mesh *> &meshes);

private:
    void addTriangle(const Triangle &triangle);

private:
    static const int MAX_DEPTH = MaxDepth;
    static const int MAX_NODE_COUNT = std::pow(2, MaxDepth);
    BoundingBox3f m_bbox;
    static OctNode m_nodes[MAX_NODE_COUNT];
};

template <int MaxDepth>
class OctreeVisitor
{
public:
    OctreeVisitor(const Octree<MaxDepth> &octree);

private:
    int m_depth;
    int m_path;
    int m_nodeIndex;
};

NORI_NAMESPACE_END