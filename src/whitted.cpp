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

        Color3f result(0.0f);
        const Mesh *mesh = its.mesh;
        if (mesh->isEmitter())
        {
            const Emitter *emitter = mesh->getEmitter();
            if (emitter)
            {
                const Ray3f emittingRay = Ray3f(its.p, (its.p - ray.o).normalized());
                result = emitter->Le(emittingRay);
            }
        }
        else if (const BSDF *bsdf = mesh->getBSDF())
        {
            for (const auto &m : scene->getMeshes())
            {
                if (!m->isEmitter() || m == mesh)
                    continue;
                result += sampleIndirectLight(scene, sampler, ray,
                                              its, m->getEmitter());
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

        if (bsdf->isDiffuse())
        {
            Intersection sampleIts;
            emitter->sampleSurface(sampler->next2D(), sampleIts);

            // Incident vector
            Vector3f incident = sampleIts.p - its.p;
            float distance = incident.norm();
            Vector3f incidentDir = incident / distance;

            // Check if sampled point is occluded
            Ray3f shadowRay(sampleIts.p, -incidentDir);
            shadowRay.maxt = distance - Epsilon;
            if (scene->rayIntersect(shadowRay))
                return Color3f(0.0f); // Occluded, return black

            // Reflection vector
            Vector3f reflect = ray.o - its.p;
            Vector3f reflectDir = reflect.normalized();

            Color3f reflectance = Color3f(0.0f);
            Frame itsFrame(its.shFrame.n);
            BSDFQueryRecord query(itsFrame.toLocal(incidentDir), itsFrame.toLocal(reflectDir), ESolidAngle);
            reflectance = bsdf->eval(query);

            Color3f li = reflectance * emitter->Le(shadowRay);
            li *= std::max(0.0f, its.shFrame.n.dot(incidentDir));
            li *= std::max(0.0f, sampleIts.shFrame.n.dot(-incidentDir));
            li /= distance * distance;

            return li;
        }
        else if (sampler->next1D() < 0.95f)
        {
            Frame itsFrame(its.shFrame.n);
            Vector3f incident = ray.o - its.p;
            Vector3f incidentDir = incident.normalized();
            BSDFQueryRecord query(itsFrame.toLocal(incidentDir));
            Color3f sample = bsdf->sample(query, sampler->next2D());
            if (sample.isZero())
                return Color3f(0.0f);
            // Reflection vector
            Vector3f reflectDir = itsFrame.toWorld(query.wo);

            Ray3f reflectRay(its.p, reflectDir);

            return (1.0f / 0.95f) * Li(scene, sampler, reflectRay);
        }
    }
};

NORI_REGISTER_CLASS(WhittedIntegrator, "whitted");
NORI_NAMESPACE_END