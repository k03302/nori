#include <nori/emitter.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

// Define static members
DiscretePDF Emitter::m_surfacePdf;
std::vector<Emitter *> Emitter::m_emitters;
bool Emitter::m_bSampleSurfaceInitialized = false;

void Emitter::setParent(NoriObject *parent)
{
    m_mesh = dynamic_cast<Mesh *>(parent);
    if (m_mesh == nullptr)
        throw NoriException("Emitter::setParent() expects a Mesh object as parent!");
    float totalSurfaceArea = m_mesh->totalSurfaceArea();
    m_surfacePdf.append(totalSurfaceArea);
    m_surfaceIndex = m_surfacePdf.size() - 1;
    m_emitters.push_back(this);
}

void Emitter::sampleSurface(const Point2f &sample, Intersection &its) const
{
    if (m_mesh == nullptr)
        throw NoriException("Emitter::sampleSurface() called on an emitter without a parent mesh!");

    m_mesh->sampleSurface(sample, its);
}

Color3f Emitter::getDirectLightTo(const Scene *scene, const Ray3f &ray, const Intersection &emitterIts) const
{
    if (m_mesh == nullptr)
    {
        return Color3f(0.0f); // Not intersecting this emitter's mesh
    }
    Ray3f r(emitterIts.p, -ray.d);
    return Le(r);
}

Color3f Emitter::getDirectLightTo(const Scene *scene, const Ray3f &ray) const
{
    Intersection its;

    if (!scene->rayIntersect(ray, its))
        return Color3f(0.0f); // No intersection, return black

    if (its.mesh != m_mesh)
        return Color3f(0.0f); // Not intersecting this emitter's mesh

    return getDirectLightTo(scene, ray, its);
}

Color3f Emitter::getEmittedLightTo(const Scene *scene, const Intersection &originIts, const Intersection &emitterIts) const
{
    Vector3f lightDir = originIts.p - emitterIts.p;
    float distance = lightDir.norm();
    Vector3f lightDirNormalized = lightDir / distance;

    Ray3f shadowRay(originIts.p, -lightDirNormalized);
    shadowRay.maxt = distance - Epsilon;

    if (scene->rayIntersect(shadowRay))
        return Color3f(0.0f); // Occluded, return black

    Color3f emittedLight = Le(shadowRay);
    float cosTheta1 = std::fabs(originIts.shFrame.n.dot(-lightDirNormalized));
    float cosTheta2 = std::fabs(emitterIts.shFrame.n.dot(lightDirNormalized));

    return emittedLight * cosTheta1 * cosTheta2 / (distance * distance);
}

Color3f Emitter::sampleEmittedLightTo(const Scene *scene, const Point2f &sample, const Intersection &originIts, Intersection &emitterIts) const
{
    sampleSurface(sample, emitterIts);
    return getEmittedLightTo(scene, originIts, emitterIts);
}

Color3f Emitter::Le(const Ray3f &ray) const
{
    // Default implementation returns black
    return Color3f(0.0f);
}

void Emitter::sampleSurfaceAll(const Point2f &sample, Intersection &its)
{
    its = Intersection();

    const Emitter *emitter = sampleEmitter(sample.x());

    if (emitter == nullptr)
        return;

    emitter->sampleSurface(sample, its);
}

const Emitter *Emitter::sampleEmitter(const float &sample)
{
    if (!m_bSampleSurfaceInitialized)
    {
        m_bSampleSurfaceInitialized = true;
        m_surfacePdf.normalize();
    }
    if (m_emitters.empty())
        return nullptr;

    int index = m_surfacePdf.sample(sample);
    return m_emitters[index];
}

NORI_NAMESPACE_END