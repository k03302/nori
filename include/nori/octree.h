#pragma once
#include <nori/common.h>
#include <nori/bbox.h>
NORI_NAMESPACE_BEGIN


static const int TOTAL_ELEMENT_COUNT = 10;
class NodeAlloc nodeAlloc;

typedef struct Element
{
    int fIndex;     // face(triangle) index
    Mesh* mesh;     // pointer to the mesh that has the traiangle
} Element;

class LeafNode
{
public:
    bool isFull() { return elementCount == TOTAL_ELEMENT_COUNT; }
    void initialize() { elementCount = 0; }
    bool push(Element e) {
        if(isFull()) return false;
        elements[elementCount++] = e;
    }

    

private:
    Element elements[TOTAL_ELEMENT_COUNT];
    int elementCount = 0;

};


class OctNode
{
public:
    const static int CHILD_COUNT = 8;
    
    bool isLeaf()
    {
        return bIsLeaf;
    }

    OctNode* push(const Element& element)
    {
        if(isLeaf())
        {
            if(elementNo < TOTAL_ELEMENT_COUNT)
            {
                //(pElementArray*)[elementNo++] = element;
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

    void pushToChildren(const Element& element)
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
    OctNode *children[CHILD_COUNT];
    Element (*pElementArray)[TOTAL_ELEMENT_COUNT];
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