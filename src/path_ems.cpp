#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/sampler.h>
#include <nori/bsdf.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

class PathEmsIntegrator : public Integrator
{
public:
    PathEmsIntegrator(const PropertyList &props)
    {
        /* No parameters this time */
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &_ray) const
    {
        Ray3f ray = _ray;
        Intersection its, last_its;
        float eta = 1.0f;
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

            last_its = its;
            if (!scene->rayIntersect(ray, its))
            {
                throughput = Color3f(0.0f);
                break;
            }

            if (const Emitter *emitter = its.mesh->getEmitter())
            {
                const Mesh *bsdfMesh = last_its.mesh;
                if (iteration == 0)
                {
                    resultColor += throughput * emitter->Le();
                }
                else if (bsdfMesh != nullptr)
                {
                    if (const BSDF *bsdf = bsdfMesh->getBSDF())
                    {
                        if (!bsdf->isDiffuse())
                        {
                            resultColor += throughput * emitter->Le();
                        }
                    }
                }
            }

            if (const BSDF *bsdf = its.mesh->getBSDF())
            {
                if (bsdf->isDiffuse())
                {
                    for (const auto &emitter : Emitter::getEmitters())
                    {
                        float emitterPdf, cosTheta;
                        Intersection emitterIts;
                        if (!emitter->sampleEmitter(scene, sampler->next2D(), its, emitterIts, emitterPdf, cosTheta))
                            continue;

                        Frame itsFrame = its.shFrame;
                        Vector3f wi = itsFrame.toLocal((emitterIts.p - its.p).normalized());
                        Vector3f wo = itsFrame.toLocal((ray.o - its.p).normalized());

                        BSDFQueryRecord query(wi, wo, ESolidAngle);
                        Color3f _throughput = bsdf->eval(query);
                        resultColor += throughput * _throughput * emitter->Le() * (cosTheta / emitterPdf);
                    }
                }

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
        return "PathEmsIntegrator[]";
    }
};

NORI_REGISTER_CLASS(PathEmsIntegrator, "path_ems");
NORI_NAMESPACE_END