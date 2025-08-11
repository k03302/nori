#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/sampler.h>
#include <nori/bsdf.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

class WhittedIntegrator : public Integrator
{
public:
    WhittedIntegrator(const PropertyList &props)
    {
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
    {
        // Find the surface that is visible in the requested direction
        Intersection its;
        if (!scene->rayIntersect(ray, its))
        {
            return Color3f(0.0f); // No intersection, return black
        }

        const Mesh *mesh = its.mesh;
        if (!mesh)
        {
            return Color3f(0.0f); // No mesh, return black
        }

        Color3f result(0.0f);
        // When the intersection point is an emitter
        if (const Emitter *emitter = mesh->getEmitter())
        {
            result += emitter->Le();
        }
        // When the intersection point is not an emitter
        else if (const BSDF *bsdf = mesh->getBSDF())
        {
            for (const auto &emitter : Emitter::getEmitters())
            {
                result += sampleIndirectLight(scene, sampler, ray,
                                              its, emitter);
            }
        }

        return result;
    }

    std::string toString() const
    {
        return "WhittedIntegrator[]";
    }

private:
    /*
        Samples the indirect lighting contribution from a given emitter to the intersection point.
        its -
    */
    Color3f sampleIndirectLight(const Scene *scene, Sampler *sampler, const Ray3f &ray,
                                const Intersection &its, const Emitter *emitter) const
    {
        if (!its.mesh)
            return Color3f(0.0f); // No mesh, return black
        const BSDF *bsdf = its.mesh->getBSDF();
        if (!bsdf)
            return Color3f(0.0f); // No BSDF, return black

        if (sampler->next1D() >= 0.95f)
            return Color3f(0.0f); // Russian roulette failed

        Color3f resultColor(0.0f);

        if (bsdf->isDiffuse())
        {
            float emitterPdf;
            Intersection emitterIts;
            bool success = emitter->sampleEmitter(sampler->next2D(), its, emitterIts, emitterPdf);

            // Incident vector
            Vector3f incident = emitterIts.p - its.p;
            Vector3f incidentDir = incident.normalized();
            Ray3f incidentRay(emitterIts.p, -incidentDir);
            success &= scene->rayIntersect(incidentRay);

            if (!success)
                return Color3f(0.0f);

            // Reflection vector
            Vector3f reflect = ray.o - its.p;
            Vector3f reflectDir = reflect.normalized();

            Color3f throughput = Color3f(0.0f);
            Frame itsFrame = its.shFrame;
            BSDFQueryRecord query(itsFrame.toLocal(incidentDir), itsFrame.toLocal(reflectDir), ESolidAngle);
            throughput = bsdf->eval(query);

            Ray3f reflectRay(its.p, incidentDir);
            resultColor = throughput * Li(scene, sampler, reflectRay) / emitterPdf;
        }
        else
        {
            // Incident vector
            Vector3f incident = ray.o - its.p;
            Vector3f incidentDir = incident.normalized();

            // Sample the BSDF
            Frame itsFrame = its.shFrame;
            BSDFQueryRecord query(itsFrame.toLocal(incidentDir));
            Color3f sample = bsdf->sample(query, sampler->next2D());
            if (sample.isZero())
                return Color3f(0.0f);

            // Reflection vector
            Vector3f reflectDir = itsFrame.toWorld(query.wo);

            // Recursive
            Ray3f reflectRay(its.p, reflectDir);
            resultColor = sample * Li(scene, sampler, reflectRay);
        }

        return (1.0f / 0.95f) * resultColor;
    }
};

NORI_REGISTER_CLASS(WhittedIntegrator, "whitted");
NORI_NAMESPACE_END