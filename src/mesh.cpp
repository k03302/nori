/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/mesh.h>
#include <nori/bbox.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>
#include <nori/warp.h>
#include <Eigen/Geometry>

NORI_NAMESPACE_BEGIN

Mesh::Mesh() {}

Mesh::~Mesh()
{
    delete m_bsdf;
    delete m_emitter;
}

void Mesh::activate()
{
    if (!m_bsdf)
    {
        /* If no material was assigned, instantiate a diffuse BRDF */
        m_bsdf = static_cast<BSDF *>(
            NoriObjectFactory::createInstance("diffuse", PropertyList()));
    }
    initSurfacePdf();
}

void Mesh::initSurfacePdf()
{
    m_surfacePdf.clear();
    m_surfacePdf.reserve(getTriangleCount());
    for (int i = 0; i < getTriangleCount(); i++)
    {
        m_surfacePdf.append(surfaceArea(i));
    }
    m_surfacePdf.normalize();
    m_bSurfacePdfInitialized = true;
}

void Mesh::sampleTriangle(uint32_t f_index, const Point2f &sample, Intersection &its) const
{
    float a = 1.0 - std::sqrt(1.0 - sample.x());
    float b = sample.y() * std::sqrt(1.0 - sample.x());
    float r = 1.0 - a - b;

    int i = m_F(0, f_index);
    int j = m_F(1, f_index);
    int k = m_F(2, f_index);

    // Assuming m_V is not empty
    Vector3f u = m_V.col(i);
    Vector3f v = m_V.col(j);
    Vector3f w = m_V.col(k);

    its.p = u * a + v * b + w * r;

    Normal3f geo_n = (v - u).cross(w - u);
    geo_n.normalize();
    its.geoFrame = Frame(geo_n);

    if (m_N.size() > 0)
    {
        Normal3f n = m_N.col(i) * a + m_N.col(j) * b + m_N.col(k) * r;
        n.normalize();
        its.shFrame = Frame(n);
    }
    else
    {
        its.shFrame = its.geoFrame; // No normals, use geometric frame
    }

    Point2f uv;
    if (m_UV.size() > 0)
    {
        uv = m_UV.col(i) * a + m_UV.col(j) * b + m_UV.col(k) * r;
    }
    its.uv = uv;
    its.t = 0.0f; // Unoccluded distance along the ray
    its.mesh = this;
}

void Mesh::sampleSurface(const Point2f &sample, Intersection &its)
{
    if (!m_bSurfacePdfInitialized)
    {
        initSurfacePdf();
    }
    size_t index = m_surfacePdf.sample(sample.x());

    sampleTriangle((uint32_t)index, sample, its);
}

float Mesh::surfaceArea(uint32_t index) const
{
    uint32_t i0 = m_F(0, index), i1 = m_F(1, index), i2 = m_F(2, index);

    const Point3f p0 = m_V.col(i0), p1 = m_V.col(i1), p2 = m_V.col(i2);

    return 0.5f * Vector3f((p1 - p0).cross(p2 - p0)).norm();
}

bool Mesh::rayIntersect(uint32_t index, const Ray3f &ray, float &u, float &v, float &t) const
{
    uint32_t i0 = m_F(0, index), i1 = m_F(1, index), i2 = m_F(2, index);
    const Point3f p0 = m_V.col(i0), p1 = m_V.col(i1), p2 = m_V.col(i2);

    /* Find vectors for two edges sharing v[0] */
    Vector3f edge1 = p1 - p0, edge2 = p2 - p0;

    /* Begin calculating determinant - also used to calculate U parameter */
    Vector3f pvec = ray.d.cross(edge2);

    /* If determinant is near zero, ray lies in plane of triangle */
    float det = edge1.dot(pvec);

    if (det > -1e-8f && det < 1e-8f)
        return false;
    float inv_det = 1.0f / det;

    /* Calculate distance from v[0] to ray origin */
    Vector3f tvec = ray.o - p0;

    /* Calculate U parameter and test bounds */
    u = tvec.dot(pvec) * inv_det;
    if (u < 0.0 || u > 1.0)
        return false;

    /* Prepare to test V parameter */
    Vector3f qvec = tvec.cross(edge1);

    /* Calculate V parameter and test bounds */
    v = ray.d.dot(qvec) * inv_det;
    if (v < 0.0 || u + v > 1.0)
        return false;

    /* Ray intersects triangle -> compute t */
    t = edge2.dot(qvec) * inv_det;

    return t >= ray.mint && t <= ray.maxt;
}

BoundingBox3f Mesh::getBoundingBox(uint32_t index) const
{
    BoundingBox3f result(m_V.col(m_F(0, index)));
    result.expandBy(m_V.col(m_F(1, index)));
    result.expandBy(m_V.col(m_F(2, index)));
    return result;
}

Point3f Mesh::getCentroid(uint32_t index) const
{
    return (1.0f / 3.0f) *
           (m_V.col(m_F(0, index)) +
            m_V.col(m_F(1, index)) +
            m_V.col(m_F(2, index)));
}

void Mesh::addChild(NoriObject *obj)
{
    switch (obj->getClassType())
    {
    case EBSDF:
        if (m_bsdf)
            throw NoriException(
                "Mesh: tried to register multiple BSDF instances!");
        m_bsdf = static_cast<BSDF *>(obj);
        break;

    case EEmitter:
    {
        Emitter *emitter = static_cast<Emitter *>(obj);
        if (m_emitter)
            throw NoriException(
                "Mesh: tried to register multiple Emitter instances!");
        m_emitter = emitter;
    }
    break;

    default:
        throw NoriException("Mesh::addChild(<%s>) is not supported!",
                            classTypeName(obj->getClassType()));
    }
}

std::string Mesh::toString() const
{
    return tfm::format(
        "Mesh[\n"
        "  name = \"%s\",\n"
        "  vertexCount = %i,\n"
        "  triangleCount = %i,\n"
        "  bsdf = %s,\n"
        "  emitter = %s\n"
        "]",
        m_name,
        m_V.cols(),
        m_F.cols(),
        m_bsdf ? indent(m_bsdf->toString()) : std::string("null"),
        m_emitter ? indent(m_emitter->toString()) : std::string("null"));
}

std::string Intersection::toString() const
{
    if (!mesh)
        return "Intersection[invalid]";

    return tfm::format(
        "Intersection[\n"
        "  p = %s,\n"
        "  t = %f,\n"
        "  uv = %s,\n"
        "  shFrame = %s,\n"
        "  geoFrame = %s,\n"
        "  mesh = %s\n"
        "]",
        p.toString(),
        t,
        uv.toString(),
        indent(shFrame.toString()),
        indent(geoFrame.toString()),
        mesh ? mesh->toString() : std::string("null"));
}

NORI_NAMESPACE_END
