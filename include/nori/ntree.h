/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#pragma once

#include <nori/common.h>
#include <nori/mesh.h>
#include <queue>

NORI_NAMESPACE_BEGIN

class Triangle
{
public:
    int m_fIndex;
    const Mesh *m_mesh;

private:
    BoundingBox3f m_bbox;

public:
    const BoundingBox3f &getBoundingBox() const
    {
        return m_bbox;
    }

    const static int maxCount = 1000000;
    static Triangle m_pool[maxCount];
    static int m_poolIndex;

    static Triangle *createTriangleData(int fIndex, const Mesh *mesh);

    void calculateBoundingBox();
};

class TrianglePtr
{
private:
    const Triangle *m_triangle;
    TrianglePtr *m_next;

public:
    const static int maxCount = 10000000;
    static TrianglePtr m_pool[maxCount];
    static int m_poolIndex;

    static TrianglePtr* createTrianglePtr(const Triangle* triangle);
    void setNextTriangle(const Triangle* nextTriangle);

    const Triangle *getTriangle() const
    {
        return m_triangle;
    }

    TrianglePtr *getNext()
    {
        return m_next;
    }

    const TrianglePtr *getNext() const
    {
        return m_next;
    }
};


struct TriangleLinkedList
{
private:
    int count = 0;
    TrianglePtr *head = nullptr;
    TrianglePtr *tail = nullptr;

public:
    int getCount() const
    {
        return count;
    }

    const TrianglePtr* getHead() const
    {
        return head;
    }

    void addTriangle(const Triangle* triangle);
};

static constexpr int pow(int base, int exp)
{
    int result = 1;
    for (int i = 0; i < exp; ++i)
    {
        result *= base;
    }
    return result;
}

class OctnodeState
{
public:
    int m_path;
    int m_treePos;
    int m_depth;
    BoundingBox3f m_bbox;
};

template <int MaxDepth>
class Octree
{
private:
    BoundingBox3f m_bbox;
    int m_totalTriangle;
    int m_actualMaxDepth;
    std::vector<Mesh *> m_meshes;

    static const int MAX_DEPTH = MaxDepth; // Maximum depth of the nonleaf nodes
    static const int CHILD_COUNT = 8;
    static const int SHIFT_COUNT = 3;                                // log2(CHILD_COUNT)
    static const unsigned int TOTAL_LEAF_COUNT = pow(CHILD_COUNT, MAX_DEPTH); // Total number of leaf nodes
	static const unsigned int TOTAL_NODE_COUNT = pow(CHILD_COUNT, MAX_DEPTH + 1); // Total number of nodes in the octree (including nonleaf nodes)
    TriangleLinkedList m_triangleLists[TOTAL_LEAF_COUNT];                    // leaf nodes (each last nonleaf node has one leaf node)
	int m_triangleCount[TOTAL_NODE_COUNT] = { 0 }; // Count of triangles included in sub octant

public:
    void build(std::vector<Mesh*> meshes);

    bool rayIntersect(Ray3f& ray, Intersection& its, uint32_t& fIndex, bool shadowRay);

    void printStatistic();

    TriangleLinkedList* getTriangleList(int pathToLeaf)
    {
        return &m_triangleLists[pathToLeaf];
    }

	int getTriangleCount(int nodeIndex) const
	{
		return m_triangleCount[nodeIndex];
	}

private:
	// Add a mesh to the octree. Internal use.
    void addMesh(Mesh* mesh);

	// Add a triange to the octree. Internal use.
	
	// depth = the current depth in the octree
    // path = oct number representing the path to the leaf node
	// (ex: oct number 1735 means the path to the leaf node was through 1st node -> 7th node -> 3rd node -> 5th node)
    void addTriangle(const Triangle* triangle, const BoundingBox3f& bbox, int depth = 0, int path = 0, int treePos = 0);
};

NORI_NAMESPACE_END
