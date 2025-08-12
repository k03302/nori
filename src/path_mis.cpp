#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/sampler.h>
#include <nori/bsdf.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

class PathMisIntegrator : public Integrator
{
public:
    PathMisIntegrator(const PropertyList &props)
    {
        /* No parameters this time */
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &_ray) const
    {
        Ray3f ray = _ray;
        Ray3f last_ray = _ray;
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
                float weight = 1.0f;

                if (iteration != 0 && bsdfMesh != nullptr)
                {
                    // Calculate the weight for MIS
                    if (const BSDF *bsdf = bsdfMesh->getBSDF())
                    {
                        if (bsdf->isDiffuse())
                        {
                            Frame itsFrame = last_its.shFrame;
                            Vector3f wi = itsFrame.toLocal(ray.d);
                            Vector3f wo = itsFrame.toLocal(-last_ray.d);
                            BSDFQueryRecord rec(wi, wo, ESolidAngle);
                            const float bsdfPdf = bsdf->pdf(rec);
                            const float emitterPdf = emitter->pdf(scene, last_its, its);
                            weight = bsdfPdf / (bsdfPdf + emitterPdf);
                        }
                    }
                }

                resultColor += weight * throughput * emitter->Le();
            }

            last_ray = ray;
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

                        const float bsdfPdf = bsdf->pdf(query);
                        const float weight = emitterPdf / (emitterPdf + bsdfPdf);

                        resultColor += weight * throughput * _throughput * emitter->Le() * (cosTheta / emitterPdf);
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
        return "PathMisIntegrator[]";
    }
};

NORI_REGISTER_CLASS(PathMisIntegrator, "path_mis");
NORI_NAMESPACE_END