#include <nori/ntree.h>
#include <nori/mesh.h>
#include <queue>

NORI_NAMESPACE_BEGIN

Triangle *Triangle::createTriangleData(int fIndex, const Mesh *mesh)
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

void Triangle::calculateBoundingBox()
{
    if (m_mesh == nullptr)
        return;
    m_bbox = m_mesh->getBoundingBox(m_fIndex);
}

Triangle Triangle::m_pool[Triangle::maxCount];
int Triangle::m_poolIndex = 0;




TrianglePtr* TrianglePtr::createTrianglePtr(const Triangle* triangle)
{
    if (m_poolIndex >= maxCount)
    {
        throw std::runtime_error("TrianglePtr pool exhausted");
    }
    TrianglePtr* ptr = &m_pool[m_poolIndex++];
    ptr->m_triangle = triangle;
    return ptr;
}

void TrianglePtr::setNextTriangle(const Triangle* nextTriangle)
{
	m_next = createTrianglePtr(nextTriangle);
}

TrianglePtr TrianglePtr::m_pool[TrianglePtr::maxCount];
int TrianglePtr::m_poolIndex = 0;








void TriangleLinkedList::addTriangle(const Triangle* triangle)
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


template<int MaxDepth>
inline void Octree<MaxDepth>::build(std::vector<Mesh*> meshes)
{
    m_meshes = meshes;
    m_bbox = m_meshes[0]->getBoundingBox();
    for (const auto& mesh : m_meshes)
    {
        m_bbox.expandBy(mesh->getBoundingBox());
    }
    for (const auto& mesh : m_meshes)
    {
        addMesh(mesh);
    }

    m_actualMaxDepth = std::min(MaxDepth, static_cast<int>(std::ceil(std::log2(m_totalTriangle))));
}

template<int MaxDepth>
bool Octree<MaxDepth>::rayIntersect(Ray3f& ray, Intersection& its, uint32_t& fIndex, bool shadowRay)
{
    bool foundIntersection = false; // Was an intersection found so far?
    fIndex = (uint32_t)-1;          // Triangle index of the closest intersection

    std::queue<OctnodeState> queue;
    queue.push({ 0, 0, 0, m_bbox }); // Start with the root node
    while (!queue.empty())
    {
        auto state = queue.front();
        queue.pop();

        if (state.m_depth == MAX_DEPTH)
        {
            auto triangleList = getTriangleList(state.m_path);
            for (const TrianglePtr* ptr = triangleList->getHead(); ptr != nullptr; ptr = ptr->getNext())
            {
                const Triangle* triangle = ptr->getTriangle();
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
				int octantPos = (state.m_treePos << SHIFT_COUNT) + i + 1; // Calculate the position in the triangle count array
                int octantPath = (state.m_path << SHIFT_COUNT) | i; // Calculate the child path
				if (getTriangleCount(octantPos) == 0)
					continue; // Skip empty octant
				float nearT, farT;
                m_bbox.getOctant(i, octant);
                if (octant.rayIntersect(ray, nearT, farT) && ray.maxt > nearT)
                {    
                    queue.push({ octantPath, octantPos, state.m_depth + 1, octant });
                }
            }
        }
    }

    return foundIntersection;
}

template<int MaxDepth>
void Octree<MaxDepth>::printStatistic()
{
    // Define histogram bins: [0], [1], [2-3], [4-7], [8-15], [16-31], [32-63], [64-127], [128-255], [256+]
    const int binCount = 10;
    int bins[binCount] = { 0 };
    int totalTriangles = 0;

    for (int i = 0; i < TOTAL_LEAF_COUNT; i++) {
        int count = m_triangleLists[i].getCount();
        totalTriangles += count;
        if (count == 0)
            bins[0]++;
        else if (count == 1)
            bins[1]++;
        else if (count <= 3)
            bins[2]++;
        else if (count <= 7)
            bins[3]++;
        else if (count <= 15)
            bins[4]++;
        else if (count <= 31)
            bins[5]++;
        else if (count <= 63)
            bins[6]++;
        else if (count <= 127)
            bins[7]++;
        else if (count <= 255)
            bins[8]++;
        else
            bins[9]++;
    }

    std::cout << "Triangle count per node histogram:" << std::endl;
    std::cout << "  0        : " << bins[0] << std::endl;
    std::cout << "  1        : " << bins[1] << std::endl;
    std::cout << "  2-3      : " << bins[2] << std::endl;
    std::cout << "  4-7      : " << bins[3] << std::endl;
    std::cout << "  8-15     : " << bins[4] << std::endl;
    std::cout << " 16-31     : " << bins[5] << std::endl;
    std::cout << " 32-63     : " << bins[6] << std::endl;
    std::cout << " 64-127    : " << bins[7] << std::endl;
    std::cout << "128-255    : " << bins[8] << std::endl;
    std::cout << "256+       : " << bins[9] << std::endl;
    std::cout << "Total triangles: " << totalTriangles << std::endl;
}

template<int MaxDepth>
void Octree<MaxDepth>::addMesh(Mesh* mesh)
{
    if (mesh == nullptr)
        return;

	int triangleCount = mesh->getTriangleCount();

	m_totalTriangle += triangleCount;

    for (uint32_t i = 0; i < triangleCount; ++i)
    {
        const Triangle* triangle = Triangle::createTriangleData(i, mesh);
        if (triangle != nullptr)
        {
            addTriangle(triangle, m_bbox, 0, 0, 0);
        }
    }
}

template<int MaxDepth>
void Octree<MaxDepth>::addTriangle(const Triangle* triangle, const BoundingBox3f& bbox, int depth, int path, int treePos)
{
    if (triangle == nullptr)
    {
        throw std::runtime_error("Invalid triangle data");
    }

    m_triangleCount[treePos]++; // Increment the triangle count for this octant
    if (depth == MaxDepth)
    {
        if (path >= TOTAL_LEAF_COUNT)
        {
            throw std::runtime_error("Path exceeds total leaf count");
        }
        m_triangleLists[path].addTriangle(triangle);
        return;
    }

    BoundingBox3f octant;
    for (int i = 0; i < CHILD_COUNT; ++i)
    {
        m_bbox.getOctant(i, octant);
        if (octant.overlaps(triangle->getBoundingBox()))
        {
            int childPath = (path << SHIFT_COUNT) | i; // Calculate the child path
			int childPos = (treePos << SHIFT_COUNT) + i + 1; // Calculate the position in the triangle count array
            addTriangle(triangle, octant, depth + 1, childPath, childPos);
        }
    }
}

template class Octree<0>;
template class Octree<1>;
template class Octree<2>;
template class Octree<3>;
template class Octree<4>;
template class Octree<5>;
template class Octree<6>;
template class Octree<7>;
template class Octree<8>;

NORI_NAMESPACE_END