#include <nori/octree.h>
NORI_NAMESPACE_BEGIN

int Triangle::currentTriangleCount = 0;
Triangle Triangle::triangleStack[TRIANGLE_COUNT];

int OctNode::currentOctNodeCount = 0;
OctNode OctNode::octNodeStack[MAX_OCTNODE_COUNT];

template <int MaxDepth>
OctNode *Octree<MaxDepth>::m_nodes[MAX_NODE_COUNT];

template <int MaxDepth>
Octree<MaxDepth> Octree<MaxDepth>::instance;

template <int MaxDepth>
int OctreeVisitor<MaxDepth>::omittedTriangleCount = 0;

template <int MaxDepth>
void Octree<MaxDepth>::build(const std::vector<Mesh *> &meshes)
{
    for (const Mesh *mesh : meshes)
    {
        m_bbox.expandBy(mesh->getBoundingBox());
    }

    // expand m_bbox as biggest cube that contains all mesh bboxes
    m_bbox.min = Vector3f::Constant(m_bbox.min.minCoeff());
    m_bbox.max = Vector3f::Constant(m_bbox.max.maxCoeff());

    for (const Mesh *mesh : meshes)
    {
        addMesh(mesh);
    }
}

template <int MaxDepth>
void Octree<MaxDepth>::addMesh(const Mesh *mesh)
{
    using namespace std;

    if (mesh == nullptr)
    {
        return;
    }
    for (uint32_t fIndex = 0; fIndex < mesh->getTriangleCount(); ++fIndex)
    {
        const Triangle *triangle = Triangle::getTriangle(fIndex, mesh);
        addTriangle(triangle);
        cout << '\r' << "Adding triangles to octree: " << fIndex << '/' << mesh->getTriangleCount();
    }
}

template <int MaxDepth>
void Octree<MaxDepth>::printStatus()
{
    using namespace std;
    cout << "Octree Status:" << endl;
    cout << "Max Depth: " << MAX_DEPTH << endl;
    cout << "Max Node Count: " << MAX_NODE_COUNT << endl;
    cout << "Bounding Box: " << m_bbox.min << m_bbox.max << endl;
    cout << "Omitted Count:" << m_visitor.omittedTriangleCount << endl;

    std::array<int, 8> overlapCounts = {0};
    researchTree(overlapCounts);
    for (int i = 0; i < 8; i++)
    {
        cout << "Overlap Count" << i + 1 << ": " << overlapCounts[i] << endl;
    }
}

template <int MaxDepth>
OctreeVisitor<MaxDepth>::OctreeVisitor(const Octree<MaxDepth> *octree)
    : m_octree(octree)
{
    if (!octree)
    {
        throw NoriException("OctreeVisitor: octree pointer is null");
    }
    reset();
}

template <int MaxDepth>
void OctreeVisitor<MaxDepth>::addTriangle(const Triangle *triangle)
{
    if (m_depth >= MaxDepth)
    {
        omittedTriangleCount++;
        return;
    }
    if (!m_bbox.overlaps(triangle->getBoundingBox()))
    {
        return;
    }
    OctNode &currentNode = getCurrentNode();
    OctNodeContextedTriangle contextedTriangle(triangle, m_bbox);

    if (currentNode.isFull())
    {
        OctreeVisitorSnapshot s;

        currentNode.replaceTriangle(contextedTriangle);

        snapshot(s);
        for (int i = 0; i < 8; i++)
        {
            if (contextedTriangle.m_octNodeOverlapMask & (1 << i))
            {
                visitChild(i);
                addTriangle(contextedTriangle.m_triangle);
                restoreSnapshot(s);
            }
        }
    }
    else
    {
        currentNode.addTriangle(contextedTriangle);
    }
}

template <int MaxDepth>
void OctreeVisitor<MaxDepth>::printTree(int depth, int indent)
{
    using namespace std;

    OctNode &currentNode = getCurrentNode();
    if (currentNode.isEmpty())
        return;

    currentNode.print(indent);

    if (depth > 0)
    {
        OctreeVisitorSnapshot s;
        snapshot(s);
        for (int i = 0; i < 8; i++)
        {
            visitChild(i);
            printTree(depth - 1, indent + 2);
            restoreSnapshot(s);
        }
    }
}

template <int MaxDepth>
void OctreeVisitor<MaxDepth>::researchTree(std::array<int, 8> &dist)
{
    OctNode &currentNode = getCurrentNode();
    if (currentNode.isEmpty())
        return;

    currentNode.research(dist);

    OctreeVisitorSnapshot s;
    snapshot(s);
    for (int i = 0; i < 8; i++)
    {
        visitChild(i);
        researchTree(dist);
        restoreSnapshot(s);
    }
}

void OctNode::print(int indent) const
{
    using namespace std;
    if (isEmpty())
        return;

    cout << string(indent, ' ') << "OctNode: ";
    for (int i = 0; i < 8; i++)
    {
        if (m_triangleMask & (1 << i))
        {
            cout << "(" << (m_triangles[i].m_triangle->getIndex()) << ", "
                 << m_triangles[i].m_overlapCount << "), ";
        }
    }
    cout << endl;
}

bool OctNode::addTriangle(OctNodeContextedTriangle &triangle)
{
    int mask = m_triangleMask;
    for (int i = 0; i < 8; i++)
    {
        if ((mask & (1 << i)) == 0)
        {
            m_triangles[i] = triangle;
            m_triangleMask |= (1 << i);
            return true;
        }
    }
    return false;
}

bool OctNode::replaceTriangle(OctNodeContextedTriangle &triangle)
{
    int mask = m_triangleMask;
    int minIndex = -1;
    int minOverlapCount = triangle.m_overlapCount;
    for (int i = 0; i < 8; i++)
    {
        if ((mask & (1 << i)) == 1)
        {
            if (minOverlapCount > m_triangles[i].m_overlapCount)
            {
                minIndex = i;
                minOverlapCount = m_triangles[i].m_overlapCount;
            }
        }
    }

    if (minIndex != -1)
    {
        triangle = m_triangles[minIndex];
        m_triangleMask &= ~(1 << minIndex);
        return true;
    }

    return false;
}

void OctNode::research(std::array<int, 8> &dist)
{
    for (int i = 0; i < 8; i++)
    {
        if (m_triangleMask & (1 << i))
        {
            int overlapCount = m_triangles[i].m_overlapCount;
            assert(overlapCount >= 1 && overlapCount <= 8);
            dist[overlapCount - 1]++;
        }
    }
}

void OctNodeContextedTriangle::initialize(const Triangle *triangle, const BoundingBox3f &octNodeBbox)
{
    m_triangle = triangle;
    m_octNodeOverlapMask = getOverlapMask(triangle, octNodeBbox);
    m_overlapCount = 0;
    int mask = m_octNodeOverlapMask;
    while (mask)
    {
        m_overlapCount += mask & 1;
        mask >>= 1;
    }
    if (m_overlapCount == 0)
    {
        throw NoriException("OctNodeContextedTriangle::initialize: triangle does not overlap with octree node");
    }
}

int OctNodeContextedTriangle::getOverlapMask(const Triangle *triangle, const BoundingBox3f &octNodeBbox)
{
    Point3f min = triangle->m_bbox.min;
    Point3f max = triangle->m_bbox.max;
    Point3f lowerBound = octNodeBbox.min;
    Point3f upperBound = octNodeBbox.max;

    const int xMask = getOverlapMask1D(min.x(), max.x(), lowerBound.x(), upperBound.x());
    const int yMask = getOverlapMask1D(min.y(), max.y(), lowerBound.y(), upperBound.y());
    const int zMask = getOverlapMask1D(min.z(), max.z(), lowerBound.z(), upperBound.z());

    // If any dimension has no overlap, return 0
    if (xMask == 0 || yMask == 0 || zMask == 0)
        return 0;

    // Build the 3D overlap mask for octree children
    // Each child is identified by 3 bits: z2 y1 x0
    // where bit 0 = x, bit 1 = y, bit 2 = z
    int mask = 0;

    for (int child = 0; child < 8; ++child)
    {
        int childX = child & 1;        // bit 0
        int childY = (child >> 1) & 1; // bit 1
        int childZ = (child >> 2) & 1; // bit 2

        // Check if triangle overlaps this specific child
        bool xOverlap = (xMask & (1 << childX)) != 0;
        bool yOverlap = (yMask & (1 << childY)) != 0;
        bool zOverlap = (zMask & (1 << childZ)) != 0;

        if (xOverlap && yOverlap && zOverlap)
        {
            mask |= (1 << child);
        }

        // #ifdef _DEBUG
        //         BoundingBox3f childBbox = octNodeBbox.getOctChild(child);
        //         BoundingBox3f triangleBbox = triangle->m_bbox;
        //         bool overlaps = triangleBbox.overlaps(childBbox);
        //         if (xOverlap && yOverlap && zOverlap != overlaps)
        //         {
        //             throw NoriException("OctNodeContextedTriangle::getOverlapMask: method implementation is wrong");
        //         }
        // #endif
    }

    return mask;
}

int OctNodeContextedTriangle::getOverlapMask1D(float minEl, float maxEl, float lowerBound, float upperBound) const
{
    // remove no overlap cases
    if (minEl >= upperBound && maxEl >= upperBound)
        return 0b00;
    if (minEl <= lowerBound && maxEl <= lowerBound)
        return 0b00;

    float center = (lowerBound + upperBound) * 0.5f;
    int overlapLower = (minEl < center) ? 1 : 0;
    int overlapUpper = (maxEl > center) ? 2 : 0;
    return overlapLower | overlapUpper;
}

template class Octree<10>;
template class OctreeVisitor<10>;

NORI_NAMESPACE_END