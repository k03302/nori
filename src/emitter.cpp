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

bool Emitter::sampleEmitter(const Scene *scene, const Point2f &sample, const Intersection &emitteeIts, Intersection &emitterIts, float &pdf, float &cosTheta) const
{
    if (m_mesh == nullptr)
        return false;

    sampleSurface(sample, emitterIts);

    pdf = this->pdf(scene, emitteeIts, emitterIts, cosTheta);

    return pdf > 0.0f;
}

bool Emitter::sampleEmitter(const Scene *scene, const Point2f &sample, const Intersection &emitteeIts, Intersection &emitterIts, float &pdf) const
{
    if (m_mesh == nullptr)
        return false;

    sampleSurface(sample, emitterIts);

    pdf = this->pdf(scene, emitteeIts, emitterIts);

    return pdf > 0.0f;
}

float Emitter::pdf(const Scene *scene, const Intersection &emitteeIts, const Intersection &emitterIts) const
{
    float totalArea = m_mesh->totalSurfaceArea();
    Vector3f toEmitter = emitterIts.p - emitteeIts.p;
    float distance = toEmitter.norm();
    Vector3f toEmitterNormalized = toEmitter / distance;

    float cosTheta1 = toEmitterNormalized.dot(emitteeIts.shFrame.n);
    if (cosTheta1 <= 0)
        return 0;

    float cosTheta2 = (-toEmitterNormalized).dot(emitterIts.shFrame.n);
    if (cosTheta2 <= 0)
        return 0;

    Ray3f ray(emitteeIts.p, toEmitterNormalized);
    ray.maxt = distance - Epsilon;
    if (scene->rayIntersect(ray))
        return 0;

    float pdf = distance * distance / (cosTheta1 * totalArea);
    if (std::isnan(pdf))
    {
        throw NoriException("Emitter::pdf() returned NaN.");
    }
    return pdf;
}

float Emitter::pdf(const Scene *scene, const Intersection &emitteeIts, const Intersection &emitterIts, float &cosTheta2) const
{
    float totalArea = m_mesh->totalSurfaceArea();
    Vector3f toEmitter = emitterIts.p - emitteeIts.p;
    float distance = toEmitter.norm();
    Vector3f toEmitterNormalized = toEmitter / distance;

    float cosTheta1 = toEmitterNormalized.dot(emitteeIts.shFrame.n);
    if (cosTheta1 <= 0)
        return 0;

    cosTheta2 = (-toEmitterNormalized).dot(emitterIts.shFrame.n);
    if (cosTheta2 <= 0)
        return 0;

    Ray3f ray(emitteeIts.p, toEmitterNormalized);
    ray.maxt = distance - Epsilon;
    if (scene->rayIntersect(ray))
        return 0;

    float pdf = distance * distance / (cosTheta1 * totalArea);
    if (std::isnan(pdf))
    {
        throw NoriException("Emitter::pdf() returned NaN.");
    }
    return pdf;
}

Color3f Emitter::Le(const Ray3f &ray) const
{
    // Default implementation returns black
    return Color3f(0.0f);
}

NORI_NAMESPACE_END