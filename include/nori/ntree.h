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

    static Triangle *createTriangleData(int fIndex, const Mesh *mesh)
    {
        if (m_poolIndex >= maxCount)
        {
            throw std::runtime_error("Triangle pool exhausted");
        }
        Triangle *data = &m_pool[m_poolIndex++];
        data->m_fIndex = fIndex;
        data->m_mesh = mesh;
        data->calculateBoundingBox();
        return data;
    }

    void calculateBoundingBox()
    {
        if (m_mesh == nullptr)
            return;

        m_bbox = m_mesh->getBoundingBox(m_fIndex);
    }
};

class TrianglePtr
{
private:
    const Triangle *m_triangle;
    TrianglePtr *m_next;

public:
    const static int maxCount = 1000000;
    static TrianglePtr m_pool[maxCount];
    static int m_poolIndex;

    static TrianglePtr *createTrianglePtr(const Triangle *triangle)
    {
        if (m_poolIndex >= maxCount)
        {
            throw std::runtime_error("TrianglePtr pool exhausted");
        }
        TrianglePtr *ptr = &m_pool[m_poolIndex++];
        ptr->m_triangle = triangle;
        return ptr;
    }

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

    void setNextTriangle(const Triangle *nextTriangle)
    {
        if (m_next != nullptr)
        {
            throw std::runtime_error("Next triangle already set");
        }
        m_next = createTrianglePtr(nextTriangle);
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

    void addTriangle(const Triangle *triangle)
    {
        if (tail == nullptr)
        {
            tail = TrianglePtr::createTrianglePtr(triangle);
            head = tail;
        }
        else
        {
            tail->setNextTriangle(triangle);
            tail = tail->getNext();
        }
        count++;
    }

    const TrianglePtr *getHead() const
    {
        return head;
    }
};

constexpr int pow(int base, int exp)
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
    int m_depth;
    BoundingBox3f m_bbox;
};

template <int MaxDepth>
class Octree
{
private:
    BoundingBox3f m_bbox;
    std::vector<Mesh *> m_meshes;

    static const int MAX_DEPTH = MaxDepth; // Maximum depth of the nonleaf nodes
    static const int CHILD_COUNT = 8;
    static const int SHIFT_COUNT = 3;                                // log2(CHILD_COUNT)
    static const int TOTAL_LEAF_COUNT = pow(CHILD_COUNT, MAX_DEPTH); // Total number of leaf nodes
    TriangleLinkedList m_nodes[TOTAL_LEAF_COUNT];                    // leaf nodes (each last nonleaf node has one leaf node)

public:
    void build(std::vector<Mesh *> meshes)
    {
        m_meshes = meshes;
        m_bbox = m_meshes[0]->getBoundingBox();
        for (const auto &mesh : m_meshes)
        {
            m_bbox.expandBy(mesh->getBoundingBox());
        }
        for (const auto &mesh : m_meshes)
        {
            addMesh(mesh);
        }
    }

    bool rayIntersect(Ray3f &ray, Intersection &its, uint32_t &fIndex, bool shadowRay)
    {
        bool foundIntersection = false; // Was an intersection found so far?
        fIndex = (uint32_t)-1;          // Triangle index of the closest intersection

        std::queue<OctnodeState> queue;
        queue.push({0, 0, m_bbox}); // Start with the root node
        while (!queue.empty())
        {
            auto state = queue.front();
            queue.pop();

            if (state.m_depth == MAX_DEPTH)
            {
                auto triangleList = getTriangleList(state.m_path);
                for (const TrianglePtr *ptr = triangleList->getHead(); ptr != nullptr; ptr = ptr->getNext())
                {
                    const Triangle *triangle = ptr->getTriangle();
                    float u, v, t;
                    if (triangle->m_mesh->rayIntersect(triangle->m_fIndex, ray, u, v, t))
                    {
                        if (shadowRay)
                        {
                            return true; // Found an intersection for shadow ray
                        }
                        ray.maxt = its.t = t;
                        its.uv = Point2f(u, v);
                        its.mesh = triangle->m_mesh;
                        fIndex = triangle->m_fIndex;
                        foundIntersection = true;
                    }
                }
            }
            else
            {
                BoundingBox3f octant;
                for (int i = 0; i < CHILD_COUNT; ++i)
                {
                    m_bbox.getOctant(i, octant);
                    if (octant.overlaps(ray))
                    {
                        int childPath = (state.m_path << SHIFT_COUNT) | i; // Calculate the child path
                        queue.push({childPath, state.m_depth + 1, octant});
                    }
                }
            }
        }
    }

    TriangleLinkedList *getTriangleList(int nodeIndex)
    {
        if (nodeIndex < 0 || nodeIndex >= TOTAL_LEAF_COUNT)
        {
            throw std::runtime_error("Invalid node index");
        }
        return &m_nodes[nodeIndex];
    }

private:
    void addMesh(Mesh *mesh)
    {
        if (mesh == nullptr)
            return;
        for (uint32_t i = 0; i < mesh->getTriangleCount(); ++i)
        {
            const Triangle *triangle = Triangle::createTriangleData(i, mesh);
            if (triangle != nullptr)
            {
                addTriangle(triangle, m_bbox, 0, 0);
            }
        }
    }

    void addTriangle(const Triangle *triangle, const BoundingBox3f &bbox, int depth = 0, int path = 0)
    {
        if (triangle == nullptr)
        {
            throw std::runtime_error("Invalid triangle data");
        }

        if (depth == MAX_DEPTH)
        {
            if (path >= TOTAL_LEAF_COUNT)
            {
                throw std::runtime_error("Path exceeds total leaf count");
            }
            m_nodes[path].addTriangle(triangle);
            return;
        }

        BoundingBox3f octant;
        for (int i = 0; i < CHILD_COUNT; ++i)
        {
            m_bbox.getOctant(i, octant);
            if (octant.overlaps(triangle->getBoundingBox()))
            {
                int childPath = (path << SHIFT_COUNT) | i; // Calculate the child path
                addTriangle(triangle, octant, depth + 1, childPath);
            }
        }
    }
};

NORI_NAMESPACE_END
