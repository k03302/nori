#include <nori/integrator.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

class WhittedIntegrator : public Integrator
{
public:
    WhittedIntegrator(const PropertyList &props)
    {
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
    {
        return Color3f(0.0f);
    }

    std::string toString() const
    {
        return "WhittedIntegrator[]";
    }
};

NORI_REGISTER_CLASS(WhittedIntegrator, "whitted");
NORI_NAMESPACE_END