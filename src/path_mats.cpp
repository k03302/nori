#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/sampler.h>
#include <nori/bsdf.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

class PathMatsIntegrator : public Integrator
{
public:
    PathMatsIntegrator(const PropertyList &props)
    {
        /* No parameters this time */
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
    {
        Ray3f last_ray = ray;
        Intersection last_its;
        float eta = 1.0f;
        Color3f last_reflectance(1.0f);
        Color3f throughput(1.0f);
        Color3f resultColor(0.0f);

        for (int iteration = 0;; ++iteration)
        {
            if (iteration >= 3)
            {
                if (sampler->next1D() < std::fmin(0.99, eta * eta * throughput.maxCoeff()))
                {
                    break;
                }
            }

            if (!scene->rayIntersect(last_ray, last_its))
            {
                throughput = Color3f(0.0f);
                break;
            }

            if (const Emitter *emitter = last_its.mesh->getEmitter())
            {
                const Ray3f emittingRay = Ray3f(last_its.p, -last_ray.d);
                resultColor += throughput * emitter->Le(emittingRay);
            }

            if (const BSDF *bsdf = last_its.mesh->getBSDF())
            {
                BSDFQueryRecord bRec(last_its.shFrame.toLocal(-last_ray.d));
                last_reflectance = bsdf->sample(bRec, sampler->next2D());
                throughput *= last_reflectance;
                last_ray = Ray3f(last_its.p, last_its.shFrame.toWorld(bRec.wo));
                eta *= bRec.eta;
            }
            else
            {
                break;
            }
        }

        return resultColor;
    }

    std::string toString() const
    {
        return "PathMatsIntegrator[]";
    }

private:
    Color3f sampleIndirectLight(const Scene *scene, Sampler *sampler, const Ray3f &ray,
                                const Intersection &its) const
    {
        Color3f result(0.0f);
        for (const auto &m : scene->getMeshes())
        {
            if (!m->isEmitter())
                continue;
            result += sampleIndirectLight(scene, sampler, ray,
                                          its, m->getEmitter());
        }
        return result;
    }

    Color3f sampleIndirectLight(const Scene *scene, Sampler *sampler, const Ray3f &ray,
                                const Intersection &its, const Emitter *emitter) const
    {
        if (!its.mesh)
            return Color3f(0.0f); // No mesh, return black
        const BSDF *bsdf = its.mesh->getBSDF();
        if (!bsdf)
            return Color3f(0.0f); // No BSDF, return black

        if (!bsdf->isDiffuse())
            return Color3f(0.0f); // Only diffuse BSDFs are considered

        // Emitted light
        Intersection emitterIts;
        Color3f emittedLight = emitter->sampleEmittedLightTo(scene, sampler->next2D(), its, emitterIts);

        // Incident vector
        Vector3f incident = emitterIts.p - its.p;
        Vector3f incidentDir = incident.normalized();

        // Reflection vector
        Vector3f reflect = ray.o - its.p;
        Vector3f reflectDir = reflect.normalized();

        // Query throughput to bsdf
        Color3f throughput = Color3f(0.0f);
        Frame itsFrame = its.shFrame;
        BSDFQueryRecord query(itsFrame.toLocal(incidentDir), itsFrame.toLocal(reflectDir), ESolidAngle);
        throughput = bsdf->eval(query);

        return emittedLight * throughput;
    }
};

NORI_REGISTER_CLASS(PathMatsIntegrator, "path_mats");
NORI_NAMESPACE_END