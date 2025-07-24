#pragma once
#include <nori/common.h>
#include <nori/bbox.h>
#include <queue>
#include <nori/mesh.h>
NORI_NAMESPACE_BEGIN

// Allocator singletons
constexpr int TOTAL_OCT_NODE_COUNT = 5000000;
constexpr int TOTAL_LIST_COUNT = 5000000;
constexpr int TOTAL_TRIANGLE_COUNT = 10000000;
constexpr int MAX_OCTREE_DEPTH = 10;

inline void printIndent(std::ostream &os, int indent)
{
    for (int i = 0; i < indent; ++i)
        os << "    ";
}

inline BoundingBox3f getOctSubcell(const BoundingBox3f &bbox, int index)
{
    BoundingBox3f subcell;
    // For each axis, select min or max based on the bits of i
    subcell.min.x() = (index & 1) ? bbox.getCenter().x() : bbox.min.x();
    subcell.max.x() = (index & 1) ? bbox.max.x() : bbox.getCenter().x();

    subcell.min.y() = (index & 2) ? bbox.getCenter().y() : bbox.min.y();
    subcell.max.y() = (index & 2) ? bbox.max.y() : bbox.getCenter().y();

    subcell.min.z() = (index & 4) ? bbox.getCenter().z() : bbox.min.z();
    subcell.max.z() = (index & 4) ? bbox.max.z() : bbox.getCenter().z();

    return subcell;
}

// StackAllocator is a simple stack-based memory allocator for OctNode and TriangleList.
template <class Element, int Count, int MaxTmpPool = 1024>
class StackAllocator
{
private:
    static Element pool[Count];
    static int currentCount;
    static std::queue<Element *> tmpPool;

protected:
    static Element *allocate()
    {
        if (!tmpPool.empty())
        {
            Element *el = tmpPool.front();
            tmpPool.pop();
            return el;
        }
        if (currentCount >= Count)
        {
            std::cerr << "StackAllocator: Out of memory!" << typeid(Element).name() << std::endl;
            exit(1);
        }
        return &pool[currentCount++];
    }

    static void deallocate(Element *el)
    {
        if ((int)tmpPool.size() < MaxTmpPool)
        {
            tmpPool.push(el);
        }
        // else: drop the pointer, don't store more than MaxTmpPool
    }

public:
    static int getCurrentCount()
    {
        return currentCount;
    }
};

class Triangle : public StackAllocator<Triangle, TOTAL_TRIANGLE_COUNT>
{
public:
    int fIndex; // face(triangle) index
    Mesh *mesh; // pointer to the mesh that owns this triangle
    const BoundingBox3f &getBoundingBox() const { return bbox; }

    Triangle() : fIndex(-1), mesh(nullptr) {}

    Triangle(int _fIndex, Mesh *_mesh)
    {
        initialize(_fIndex, _mesh);
    }

    static Triangle *getTriangle(int _fIndex, Mesh *_mesh)
    {
        Triangle *triangle = Triangle::allocate();
        if (triangle == nullptr)
            return nullptr;
        triangle->initialize(_fIndex, _mesh);
        return triangle;
    }

    static void returnTriangle(Triangle *triangle)
    {
        if (triangle == nullptr)
            return;
        triangle->fIndex = -1;    // Reset index
        triangle->mesh = nullptr; // Reset mesh pointer
        Triangle::deallocate(triangle);
    }

    void initialize(int _fIndex, Mesh *_mesh)
    {
        fIndex = _fIndex;
        mesh = _mesh;
        calculateBoundingBox();
    }

    void calculateBoundingBox()
    {
        if (mesh == nullptr)
            return;
        bbox = mesh->getBoundingBox(fIndex);
    }

    void print(std::ostream &os, int indent) const
    {
        printIndent(os, indent);
        os << "(" << fIndex << ")\n";
    }

private:
    BoundingBox3f bbox; // bounding box of the triangle
};

class TriangleList : public StackAllocator<TriangleList, TOTAL_LIST_COUNT>
{
public:
    static const int TOTAL_ELEMENT_COUNT = 10;

    TriangleList()
    {
        initialize();
    }

    void initialize()
    {
        triangleCount = 0;
        for (int i = 0; i < TOTAL_ELEMENT_COUNT; i++)
        {
            triangles[i] = nullptr;
        }
    }

    static TriangleList *getTriangleList()
    {
        TriangleList *list = TriangleList::allocate();
        if (list == nullptr)
            return nullptr;
        list->initialize();
        return list;
    }

    static void returnTriangleList(TriangleList *list)
    {
        if (list == nullptr)
            return;
        list->initialize(); // Reset the list
        TriangleList::deallocate(list);
    }

    bool isFull() const { return triangleCount == TOTAL_ELEMENT_COUNT; }

    bool push(const Triangle &e)
    {
        if (isFull())
            return false;
        triangles[triangleCount++] = &e;
        return true;
    }

    void foreachEl(const std::function<void(const Triangle &)> &callback) const
    {
        for (int i = 0; i < triangleCount; i++)
        {
            callback(*triangles[i]);
        }
    }

    void print(std::ostream &os, int indent) const
    {
        printIndent(os, indent);
        os << "TriangleList(triangleCount=" << triangleCount << ")\n";
        for (int i = 0; i < triangleCount; i++)
        {
            triangles[i]->print(os, indent + 1);
        }
    }

private:
    const Triangle *triangles[TOTAL_ELEMENT_COUNT];
    int triangleCount;
};

class OctNode : public StackAllocator<OctNode, TOTAL_OCT_NODE_COUNT>
{
public:
    OctNode() = default;
    OctNode(const BoundingBox3f &_bbox, int _depth, bool _bIsLeaf)
    {
        initialize(_bbox, _depth, _bIsLeaf);
    }

    void initialize(const BoundingBox3f &_bbox, int _depth, bool _bIsLeaf)
    {
        bbox = _bbox;
        depth = _depth;
        bIsLeaf = _bIsLeaf;

        if (bIsLeaf)
        {
            data = TriangleList::getTriangleList();
        }
    }

    static OctNode *getOctNode(const BoundingBox3f &_bbox, int _depth, bool _bIsLeaf)
    {
        OctNode *node = OctNode::allocate();
        if (node == nullptr)
            return nullptr;
        node->initialize(_bbox, _depth, _bIsLeaf); // Initialize with default values
        return node;
    }

    static void returnOctNode(OctNode *node)
    {
        if (node == nullptr)
            return;
        OctNode::deallocate(node);
    }

    bool hasData() const
    {
        return data != nullptr;
    }

    bool isLeaf() const
    {
        return bIsLeaf;
    }

    bool push(const Triangle &triangle)
    {
        if (!isIncluding(triangle) || depth >= MAX_OCTREE_DEPTH)
        {
            return false; // Triangle does not fit in this node's bounding box
        }

        if (isLeaf())
        {

            if (!hasData())
            {
                data = TriangleList::getTriangleList();
                if (!data)
                    return false;
            }

            if (!data->isFull())
            {
                data->push(triangle);
                return true;
            }
            else
            {
                pushDataToChildren();
                return pushToChildren(triangle);
            }
        }
        else if (depth < MAX_OCTREE_DEPTH)
        {
            return pushToChildren(triangle);
        }
        return false;
    }

    void createChildWithBbox(int index, const BoundingBox3f &childBbox)
    {
        bIsLeaf = false;

        if (children[index] != nullptr)
            return; // Child already exists

        OctNode *newNode = OctNode::getOctNode(childBbox, depth + 1, true);
        if (newNode == nullptr)
            return;

        children[index] = newNode;
    }

    void createChild(int index)
    {
        bIsLeaf = false;

        if (children[index] != nullptr)
            return; // Child already exists

        BoundingBox3f childBbox = getOctSubcell(bbox, index);

        OctNode *newNode = OctNode::getOctNode(childBbox, depth + 1, true);
        if (newNode == nullptr)
            return;

        children[index] = newNode;
    }

    void pushDataToChildren()
    {
        data->foreachEl([this](const Triangle &triangle)
                        { this->pushToChildren(triangle); });
        TriangleList::returnTriangleList(data);
        data = nullptr;
    }

    bool pushToChildren(const Triangle &triangle)
    {
        if (depth >= MAX_OCTREE_DEPTH)
        {
            return false;
        }

        bool success = false;

        for (int i = 0; i < 8; i++)
        {
            OctNode *child = children[i];
            if (child == nullptr)
            {
                BoundingBox3f childBbox = getOctSubcell(bbox, i);
                if (childBbox.overlaps(triangle.getBoundingBox()))
                {
                    createChildWithBbox(i, childBbox);
                    if (children[i])
                    {
                        children[i]->push(triangle);
                        success = true;
                    }
                }
            }
            else if (child->isIncluding(triangle) && child->push(triangle))
            {
                success = true;
            }
        }

        return success;
    }

    bool isIncluding(const Triangle &triangle)
    {
        if (triangle.mesh == nullptr)
            return false;
        BoundingBox3f elementBbox = triangle.getBoundingBox();
        return elementBbox.overlaps(bbox);
    }

    void print(std::ostream &os, int indent) const
    {
        printIndent(os, indent);
        os << "OctNode(depth=" << depth << ", isLeaf=" << bIsLeaf << ")\n";
        if (hasData())
        {
            data->print(os, indent + 1);
        }
        if (!isLeaf())
        {
            for (int i = 0; i < 8; ++i)
            {
                if (children[i])
                    children[i]->print(os, indent + 1);
                else
                {
                    printIndent(os, indent + 1);
                    os << "Child " << i << " is null\n";
                }
            }
        }
    }

    const BoundingBox3f &getBoundingBox() const
    {
        return bbox;
    }

    const TriangleList *getData() const
    {
        return data;
    }

    OctNode *getChild(int index) const
    {
        if (index < 0 || index >= 8)
            return nullptr; // Invalid index
        return children[index];
    }

private:
    BoundingBox3f bbox;
    OctNode *children[8] = {nullptr};
    TriangleList *data = nullptr;
    int depth = 0;
    bool bIsLeaf = false;
};

template <>
int StackAllocator<Triangle, TOTAL_TRIANGLE_COUNT>::currentCount = 0;
template <>
Triangle StackAllocator<Triangle, TOTAL_TRIANGLE_COUNT>::pool[TOTAL_TRIANGLE_COUNT] = {Triangle()};
template <>
std::queue<Triangle *> StackAllocator<Triangle, TOTAL_TRIANGLE_COUNT>::tmpPool = std::queue<Triangle *>();

template <>
int StackAllocator<TriangleList, TOTAL_LIST_COUNT>::currentCount = 0;
template <>
TriangleList StackAllocator<TriangleList, TOTAL_LIST_COUNT>::pool[TOTAL_LIST_COUNT] = {TriangleList()};
template <>
std::queue<TriangleList *> StackAllocator<TriangleList, TOTAL_LIST_COUNT>::tmpPool = std::queue<TriangleList *>();

template <>
int StackAllocator<OctNode, TOTAL_OCT_NODE_COUNT>::currentCount = 0;
template <>
OctNode StackAllocator<OctNode, TOTAL_OCT_NODE_COUNT>::pool[TOTAL_OCT_NODE_COUNT] = {OctNode()};
template <>
std::queue<OctNode *> StackAllocator<OctNode, TOTAL_OCT_NODE_COUNT>::tmpPool = std::queue<OctNode *>();

NORI_NAMESPACE_END