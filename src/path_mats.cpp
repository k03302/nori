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

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &_ray) const
    {
        Ray3f ray = _ray;
        Intersection its;
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

            if (!scene->rayIntersect(ray, its))
            {
                throughput = Color3f(0.0f);
                break;
            }

            if (const Emitter *emitter = its.mesh->getEmitter())
            {
                resultColor += throughput * emitter->Le();
                break;
            }

            if (const BSDF *bsdf = its.mesh->getBSDF())
            {
                BSDFQueryRecord bRec(its.shFrame.toLocal(-ray.d));
                throughput *= bsdf->sample(bRec, sampler->next2D());
                ray = Ray3f(its.p, its.shFrame.toWorld(bRec.wo));

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