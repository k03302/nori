#include <nori/integrator.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

class SimpleIntegrator : public Integrator
{
public:
    SimpleIntegrator(const PropertyList &props)
    {
        props.getPoint("position", m_position);
        m_energy = props.getColor("energy");
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
    {
        /* Find the surface that is visible in the requested direction */
        Intersection first_its;
        Ray3f first_ray, bounce_ray;

        first_ray = ray;

        if (!scene->rayIntersect(first_ray, first_its))
            return Color3f(0.0f);

        
		Normal3f n = first_its.shFrame.n;
		Vector3f incident = first_ray.d;
		Point3f hitPoint = first_its.p;

        auto costheta = incident.dot(-n);
        if (costheta < 0.0f)
            return Color3f(0.0f);

        Vector3f reflect = m_position - hitPoint;

		costheta = reflect.dot(-n);
		if (costheta < 0.0f)
			return Color3f(0.0f);

        float distance = reflect.norm();
        reflect /= distance;
        bounce_ray = Ray3f(hitPoint, -reflect);
        bounce_ray.maxt = distance;

        if (scene->rayIntersect(bounce_ray))
            return Color3f(0.0f);

        Color3f color = m_energy * costheta / (4.0f * EIGEN_PI * EIGEN_PI * distance * distance);
        return color;
    }

    std::string toString() const
    {
        return "SimpleIntegrator[]";
    }

private:
    Vector3f m_position; // Position of the light source
    Color3f m_energy;    // Energy emitted by the light source
};

NORI_REGISTER_CLASS(SimpleIntegrator, "simple");
NORI_NAMESPACE_END