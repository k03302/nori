#include <nori/integrator.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

class SimpleIntegrator : public Integrator
{
public:
    SimpleIntegrator(const PropertyList &props)
    {
        props.getVector("position", m_position);
        props.getVector("energy", m_energy);
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
    {
        /* Find the surface that is visible in the requested direction */
        Intersection init_its, bounce_its;
        Ray3f init_ray(ray), bounce_ray;
        Vector3f direction;

        if (!scene->rayIntersect(init_ray, init_its))
            return Color3f(0.0f);

        direction = init_its.p - m_position;
        // bounce_ray(init_its.p, );

        // if (!scene->rayIntersect())
        // {
        //     /* If no intersection was found, return zero */
        //     return Color3f(0.0f);
        // }

        /* Return the component-wise absolute
           value of the shading normal as a color */
        // Normal3f n = its.shFrame.n.cwiseAbs();
        // return Color3f(n.x(), n.y(), n.z());
    }

    std::string toString() const
    {
        return "SimpleIntegrator[]";
    }

private:
    Vector3f m_position; // Position of the light source
    Vector3f m_energy;   // Energy emitted by the light source
};

NORI_REGISTER_CLASS(SimpleIntegrator, "simple");
NORI_NAMESPACE_END