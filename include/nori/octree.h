#pragma once
#include <nori/common.h>

NORI_NAMESPACE_BEGIN

template <typename T> class OctNode
{
public:
    const static int CHILD_COUNT = 8;
    const static int ELEMENT_COUNT = 10;
    
    OctNode* push(const T& element)
    {
        if(elementNo < ELEMENT_COUNT)
        {
            elements[elementNo++] = element;
            return this;
        }
        else
        {
            for(int i = 0; i < CHILD_COUNT; i++)
            {
                nodes[i] = 
            }
        }
    }

    
    

    OctNode() = 0;
    OctNode(const BoundingBox3d& _bbox, int _depth, shared_ptr<NodeAllocator> _allocator)
        : bbox(_bbox), depth(_depth), elementNo(0), allocator(_allocator)
    {}

private:
    BoundingBox3d bbox;
    OctNode<T>* nodes[CHILD_COUNT];
    T elements[ELEMENT_COUNT];
    int depth;
    int elementNo;
    shared_ptr<NodeAllocator> allocator;
};

template <typename T> class NodeAllocator
{
public:
    const static int NODE_COUNT = 1000000;

    NodeAllocator()
    {
        initialize();
    }

    void initialize()
    {
        nodeNo = 0;
    }

    OctNode<T>* newNode()
    {
        return &nodePool[nodeNo++];
    }

private:
    OctNode<T> nodePool[NODE_COUNT];
    int nodeNo = 0;
};



NORI_NAMESPACE_END