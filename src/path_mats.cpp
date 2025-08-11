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
                break;
            }

            if (const BSDF *bsdf = last_its.mesh->getBSDF())
            {
                BSDFQueryRecord bRec(last_its.shFrame.toLocal(-last_ray.d));
                last_reflectance = bsdf->sample(bRec, sampler->next2D());
                last_ray = Ray3f(last_its.p, last_its.shFrame.toWorld(bRec.wo));
                throughput *= last_reflectance;

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
};

NORI_REGISTER_CLASS(PathMatsIntegrator, "path_mats");
NORI_NAMESPACE_END