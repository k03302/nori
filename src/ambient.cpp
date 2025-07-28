#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

class AmbientIntegrator : public Integrator
{
public:
    AmbientIntegrator(const PropertyList &props)
    {
        /* No parameters this time */
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
    {
        /* Find the surface that is visible in the requested direction */
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);
        

		auto sampleLocal = Warp::squareToCosineHemisphere(
			sampler->next2D());
		auto hemisphereFrame = Frame(its.shFrame.n);
        auto sampleWorld = hemisphereFrame.toWorld(sampleLocal);
        sampleWorld.normalize();
        Ray3f shadowRay(its.p, sampleWorld);

        if(scene->rayIntersect(shadowRay))
			return Color3f(0.0f);
        
		float cosTheta = its.shFrame.n.dot(sampleWorld);
        return Color3f(cosTheta / EIGEN_PI);
    }

    std::string toString() const
    {
        return "AmbientIntegrator[]";
    }
};

NORI_REGISTER_CLASS(AmbientIntegrator, "ao");
NORI_NAMESPACE_END