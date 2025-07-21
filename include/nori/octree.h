#pragma once
#include <nori/common.h>
#include <nori/bbox.h>
NORI_NAMESPACE_BEGIN

class NodeAlloc nodeAlloc;

struct Element
{
    int index;
    Mesh* mesh;
};

class OctNode
{
public:
    const static int CHILD_COUNT = 8;
    const static int ELEMENT_COUNT = 10;
    
    bool isLeaf()
    {
        return bIsLeaf;
    }

    OctNode* push(const int& element)
    {
        if(isLeaf())
        {
            if(elementNo < ELEMENT_COUNT)
            {
                elements[elementNo++] = element;
                return this;
            }
            else
            {
                createChildren();
                pushToChildren(element);
                
                bIsLeaf = false;
            }
        }
        else
        {
            pushToChildren(element);
        }
    }

    void createChildren()
    {
        for(int i = 0; i < 2; i++)
        {
            for(int j = 0; j < 2; j++)
            {
                for(int k = 0; k < 2; k++)
                {
                    int index = 4 * i + 2 * j + k;

                    OctNode* newNode = nodeAlloc.getNode();
                    const Vector3d diagonal = bbox.max - bbox.min;
                    const Vector3d childDiagonal = diagonal / 2.0;
                    const Vector3i childPosVector(i, j, k);

                    const Point3d newMin = childDiagonal * childPosVector;
                    const Point3d newMax = newMin + childDiagonal;
                    BoundingBox3d newBbox(newMin, newMax);

                    children[index] = newNode;
                }
            }
        }
    }

    void pushToChildren(const int& element)
    {
        for(int i = 0; i < 2; i++)
        {
            for(int j = 0; j < 2; j++)
            {
                for(int k = 0; k < 2; k++)
                {
                    int index = 4 * i + 2 * j + k;

                    OctNode* newNode = nodeAlloc.getNode();
                    const Vector3d diagonal = bbox.max - bbox.min;
                    const Vector3d childDiagonal = diagonal / 2.0;
                    const Vector3i childPosVector(i, j, k);

                    const Point3d newMin = childDiagonal * childPosVector;
                    const Point3d newMax = newMin + childDiagonal;
                    BoundingBox3d newBbox(newMin, newMax);

                    children[index] = newNode;
                }
            }
        }

        for(int i = 0; i < 8; i++)
        {
            if (children[i] == nullptr) continue;
            children[i]->push(element);
        }
    }

    OctNode() = default;
    OctNode(const BoundingBox3d& _bbox, int _depth)
        : bbox(_bbox), depth(_depth), elementNo(0), bIsLeaf(true)
    {}

private:
    BoundingBox3d bbox;
    OctNode* children[CHILD_COUNT];
    int elements[ELEMENT_COUNT];
    int depth;
    int elementNo;
    bool bIsLeaf;
};

class NodeAlloc
{
private:
    const static int TOTAL_NODE_COUNT = 1000000;
    int nodeCount;
    OctNode nodePool[TOTAL_NODE_COUNT];
public:
    NodeAlloc() : nodeCount(0) {}
    OctNode* getNode()
    {
        return &nodePool[nodeCount++];
    }
} nodeAlloc;



NORI_NAMESPACE_END