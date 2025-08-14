#pragma once
#include <nori/common.h>
#include <nori/bbox.h>
#include <nori/mesh.h>
#include <queue>

NORI_NAMESPACE_BEGIN

struct OctreeVisitorSnapshot;
class Triangle;
struct OctNodeContextedTriangle;
class OctNode;
template <int MaxDepth>
class Octree;
template <int MaxDepth>
class OctreeVisitor;

constexpr int pow(int base, int exp)
{
    return (exp == 0) ? 1 : base * pow(base, exp - 1);
}

enum OctNodeIndex
{
    NONE = -1,
    FIRST = 0,
    X0Y0Z0 = 0,
    X1Y0Z0 = 1,
    X0Y1Z0 = 2,
    X1Y1Z0 = 3,
    X0Y0Z1 = 4,
    X1Y0Z1 = 5,
    X0Y1Z1 = 6,
    X1Y1Z1 = 7,
    END = 8
};

class Triangle
{
public:
    static const Triangle *getTriangle(int fIndex, const Mesh *mesh)
    {
        if (currentTriangleCount >= TRIANGLE_COUNT)
            throw NoriException("Triangle::getTriangle: exceeded maximum triangle count");
        Triangle &triangle = triangleStack[currentTriangleCount++];

        Vector3f v0, v1, v2;
        mesh->getVertex(fIndex, v0, v1, v2);
        triangle.m_bbox = mesh->getBoundingBox(fIndex);
        triangle.m_fIndex = fIndex;
        triangle.m_mesh = mesh;
        return &triangle;
    }

    inline int getIndex() const
    {
        return m_fIndex;
    }

	const BoundingBox3f& getBoundingBox() const
	{
		return m_bbox;
	}

private:
    int m_fIndex;
    const Mesh *m_mesh;
    BoundingBox3f m_bbox;

    static const int TRIANGLE_COUNT = 10000000;
    static Triangle triangleStack[TRIANGLE_COUNT];
    static int currentTriangleCount;

    friend class OctNodeContextedTriangle;
};

/*
Triangle context for octree nodes
*/
class OctNodeContextedTriangle
{
public:
    int m_octNodeOverlapMask;
    int m_overlapCount;
    const Triangle *m_triangle;

    const Triangle *getTriangle()
    {
        return m_triangle;
    }

    OctNodeContextedTriangle()
    {
        m_octNodeOverlapMask = 0;
        m_overlapCount = 0;
        m_triangle = nullptr;
    }
    OctNodeContextedTriangle(const Triangle *triangle, const BoundingBox3f &octNodeBbox)
    {
        initialize(triangle, octNodeBbox);
    }

    void initialize(const Triangle *triangle, const BoundingBox3f &octNodeBbox);

    /*
    This method is for calculating bitmask of inclusion of triangle bbox to octree subnode
    */
    int getOverlapMask(const Triangle *triangle, const BoundingBox3f &octNodeBbox);

    /*
    This method is for calculating bitmask of inclusion of triangle bbox to octree subnode
    This method solves 1D projected version of original problem
    This method assumes that minEl <= maxEl && lowerBound <= upperBound
    Returns bitmask (0b00: no overlap, 0b01: overlap lower subinterval,
        0b10: overlap upper subinterval, 0b11: full overlap)
    */
    int getOverlapMask1D(float minEl, float maxEl, float lowerBound, float upperBound) const;
};

class OctNode
{
public:
    static OctNode *getOctNode()
    {
        if (currentOctNodeCount >= MAX_OCTNODE_COUNT)
            throw NoriException("OctNode::getOctNode: exceeded maximum octree node count");
        return &octNodeStack[currentOctNodeCount++];
    }

    // Just add triangle to list if there's a free slot
    bool addTriangle(OctNodeContextedTriangle &triangle);

    // Replace an existing triangle to new triangle if the former has less overlap count
    bool replaceTriangle(OctNodeContextedTriangle &triangle);

    void print(int indent = 0) const;

    void research(std::array<int, 8> &dist);

    inline bool isFull() const
    {
        return m_triangleMask == 0b11111111;
    }

    inline bool isEmpty() const
    {
        return m_triangleMask == 0;
    }

private:
    static const int MAX_OCTNODE_COUNT = 1000000;
    static OctNode octNodeStack[MAX_OCTNODE_COUNT];
    static int currentOctNodeCount;

    OctNodeContextedTriangle m_triangles[8];
    int m_triangleMask = 0; // mask for tracking active triangles
};

/*
Octree is singleton
*/
template <int MaxDepth>
class Octree
{
public:
    void build(const std::vector<Mesh *> &meshes);

    void printStatus();

    void printTree(int depth)
    {
        m_visitor.reset();
        m_visitor.printTree(depth, 0);
    }

    void researchTree(std::array<int, 8> &dist)
    {
        m_visitor.reset();
        m_visitor.researchTree(dist);
    }

    static Octree<MaxDepth> &getInstance()
    {
        return instance;
    }

    void getBoundingBox(BoundingBox3f &bbox) const
    {
        bbox = m_bbox;
    }

    OctNode &getOctNode(int nodeIndex) const
    {
        if (nodeIndex < 0 || nodeIndex >= MAX_NODE_COUNT)
            throw NoriException("Octree::getOctNode: node index out of bounds");

        // lazy initialization for memory efficiency
        if (m_nodes[nodeIndex] == nullptr)
        {
            m_nodes[nodeIndex] = OctNode::getOctNode();
        }
        return *m_nodes[nodeIndex];
    }

private:
    void addTriangle(const Triangle *triangle)
    {
        m_visitor.reset();
        m_visitor.addTriangle(triangle);
    }

    void addMesh(const Mesh *mesh);

    Octree() : m_bbox(), m_visitor(this) {}

private:
    static const int MAX_DEPTH = MaxDepth;
    static const int MAX_NODE_COUNT = pow(8, MaxDepth);

    static Octree<MaxDepth> instance;

    BoundingBox3f m_bbox;
    static OctNode *m_nodes[MAX_NODE_COUNT];
    OctreeVisitor<MaxDepth> m_visitor;
};

template <int MaxDepth>
class OctreeVisitor
{
public:
    static int omittedTriangleCount;

public:
    OctreeVisitor(const Octree<MaxDepth> *octree);

    void addTriangle(const Triangle *triangle);

    void printTree(int depth, int indent = 0);

    void researchTree(std::array<int, 8> &dist);

    OctNode &getCurrentNode() const
    {
        return m_octree->getOctNode(m_nodeIndex);
    }

    inline void visitChild(int childIndex)
    {
        if (childIndex < 0 || childIndex >= 8)
            throw NoriException("OctreeVisitor::visitChild: invalid child index");
        m_path = (m_path << 3) | childIndex;
        m_depth++;
        m_nodeIndex = (m_nodeIndex << 3) | (childIndex + 1);
        m_bbox.setOctChild(childIndex);
    }

    inline void snapshot(OctreeVisitorSnapshot &snapshot)
    {
        snapshot.m_depth = m_depth;
        snapshot.m_path = m_path;
        snapshot.m_nodeIndex = m_nodeIndex;
        snapshot.m_bbox = m_bbox;
    }

    inline void restoreSnapshot(const OctreeVisitorSnapshot &snapshot)
    {
        m_depth = snapshot.m_depth;
        m_path = snapshot.m_path;
        m_nodeIndex = snapshot.m_nodeIndex;
        m_bbox = snapshot.m_bbox;
    }

    inline void reset()
    {
        m_depth = 0;
        m_path = 0;
        m_nodeIndex = 0;
        m_octree->getBoundingBox(m_bbox);
    }

private:
    const Octree<MaxDepth> *m_octree;
    int m_depth;
    int m_path;
    int m_nodeIndex;
    BoundingBox3f m_bbox;
};

struct OctreeVisitorSnapshot
{
    int m_depth;
    int m_path;
    int m_nodeIndex;
    BoundingBox3f m_bbox;
};

NORI_NAMESPACE_END