#pragma once
#include <nori/common.h>
#include <nori/bbox.h>
#include <queue>
#include <nori/mesh.h>
NORI_NAMESPACE_BEGIN


class NodeAlloc nodeAlloc;

typedef struct Triangle
{
    int fIndex;     // face(triangle) index
    Mesh* mesh;     // pointer to the mesh that owns this triangle
} Triangle;

class TriangleList
{
public:
    static const int TOTAL_ELEMENT_COUNT = 10;
    
    TriangleList() : elementCount(0) {}

    bool isFull() { return elementCount == TOTAL_ELEMENT_COUNT; }

    void empty() { elementCount = 0; }

    bool push(const Triangle& e) {
        if(isFull()) return false;
        elements[elementCount++] = e;
        return true;
    }

    void foreachEl(std::function<void(Triangle)> callback)
    {
        for(int i = 0; i < elementCount; i++)
        {
            callback(elements[i]);
        }
    }

private:
    Triangle elements[TOTAL_ELEMENT_COUNT];
    int elementCount;

};


class OctNode
{
public:
    OctNode() = default;
    OctNode(const BoundingBox3f& _bbox, int _depth, bool _bIsLeaf)
        : bbox(_bbox), depth(_depth), bIsLeaf(_bIsLeaf)
    {
        if(bIsLeaf)
        {
            data = nodeAlloc.getDataNode();
        }
    }

    bool hasData()
    {
        return data != nullptr;
    }

    bool isLeaf()
    {
        return bIsLeaf;
    }


    bool push(const Triangle& triangle)
    {
        if(isLeaf())
        {
            if(!hasData())
            {
                data = nodeAlloc.getDataNode();
                if(!data) return false;
            }

            if(!data->isFull())
            {
                data->push(triangle);
                return true;
            }
            else
            {
                createChildren();
                return pushToChildren(triangle);
            }
        }
        else
        {
            return pushToChildren(triangle);
        }
    }

    void createChildren()
    {
        bIsLeaf = false;

        for(int i = 0; i < 8; i++)
        {
            OctNode* newNode = nodeAlloc.getOctNode();
            if(newNode == nullptr) return;

            const Vector3d diagonal = bbox.max - bbox.min;
            const Vector3d childDiagonal = diagonal / 2.0;

            int x = (i | (1 << 0)) ? 1 : 0;
            int y = (i | (1 << 1)) ? 1 : 0;
            int z = (i | (1 << 2)) ? 1 : 0;

            // Vector representing the position of child on oct space
            // (0, 0, 0) = child node with bbox of bottom corner
            // (1, 1, 1) = child node with bbox of top corner
            const Vector3i childOctIndex(x, y, z);

            // Calculate the bbox of the child
            const Point3d childMin = childDiagonal * childOctIndex;
            const Point3d childMax = childMin + childDiagonal;
            BoundingBox3f newBbox(childMin, childMax);

            newNode->bbox = newBbox;
            newNode->depth = depth + 1;
            newNode->data = nodeAlloc.getDataNode();
            children[i] = newNode;
        }

        if(hasData())
        {
            data->foreachEl([this](Triangle el){
                this->pushToChildren(el);
            });
        }

        nodeAlloc.returnDataNode(data);
        data = nullptr;
    }

    bool pushToChildren(const Triangle& triangle)
    {
        bool success = false;

        for(int i = 0; i < 8; i++)
        {
            OctNode *child = children[i];
            if (child == nullptr) continue;
            if (child->isIncluding(triangle) && child->push(triangle)) success = true;
        }

        return success;
    }

    bool isIncluding(const Triangle& triangle)
    {
        if(triangle.mesh == nullptr) return false;
        BoundingBox3f elementBbox = triangle.mesh->getBoundingBox(triangle.fIndex);
        return elementBbox.overlaps(bbox);
    }

private:
    BoundingBox3f bbox;
    OctNode *children[8];
    TriangleList *data;
    int depth;
    bool bIsLeaf;
};


template <class Element, int Count> class StackAllocator
{
private:
    Element[Count] pool;
    
};


class NodeAlloc
{
private:
    const static int TOTAL_OCT_NODE_COUNT = 1000000;
    const static int TOTAL_DATA_NODE_COUNT = 1000000;
    int octNodeCount;
    int dataNodeCount;
    OctNode octNodePool[TOTAL_OCT_NODE_COUNT];
    TriangleList dataNodePool[TOTAL_DATA_NODE_COUNT];
    std::queue<OctNode*> tmpOctNodePool;
    std::queue<TriangleList*> tmpDataNodePool;

public:
    NodeAlloc() : octNodeCount(0), dataNodeCount(0) {}
    OctNode* getOctNode()
    {
        if(!tmpOctNodePool.empty()) {
            OctNode* octNode = tmpOctNodePool.front();
            tmpOctNodePool.pop();
            return octNode;
        }
        if(octNodeCount >= TOTAL_OCT_NODE_COUNT) return nullptr;
        return &octNodePool[octNodeCount++];
    }
    TriangleList* getDataNode()
    {
        if(!tmpDataNodePool.empty()) {
            TriangleList* dataNode = tmpDataNodePool.front();
            tmpDataNodePool.pop();
            return dataNode;
        }
        if(dataNodeCount >= TOTAL_DATA_NODE_COUNT) return nullptr;
        return &dataNodePool[dataNodeCount++];
    }
    void returnOctNode(OctNode* octNode)
    {
        tmpOctNodePool.push(octNode);
    }
    void returnDataNode(TriangleList* dataNode)
    {
        tmpDataNodePool.push(dataNode);
    }
} nodeAlloc;



NORI_NAMESPACE_END