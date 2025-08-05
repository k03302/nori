/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#pragma once

#include <nori/object.h>
#include <nori/mesh.h>
#include <nori/sampler.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

/**
 * \brief Superclass of all emitters
 */
class Emitter : public NoriObject
{
public:
    /**
     * \brief Return the type of object (i.e. Mesh/Emitter/etc.)
     * provided by this instance
     * */
    EClassType getClassType() const { return EEmitter; }

    void setParent(NoriObject *parent) override
    {
        m_mesh = dynamic_cast<Mesh *>(parent);
        if (m_mesh == nullptr)
            throw NoriException("Emitter::setParent() expects a Mesh object as parent!");
    }

    const Mesh *getMesh() const { return m_mesh; }

    void sampleSurface(const Point2f &sample, Intersection &its) const
    {
        if (m_mesh == nullptr)
            throw NoriException("Emitter::sampleSurface() called on an emitter without a parent mesh!");

        m_mesh->sampleSurface(sample, its);
    }

    /*
    This method assume that the ray intersects the emitter's mesh (at emitterIts) without occlusion.
    */
    Color3f getDirectLightTo(const Scene *scene, const Ray3f &ray, const Intersection &emitterIts) const
    {
        if (m_mesh != nullptr)
        {
            return Color3f(0.0f); // Not intersecting this emitter's mesh
        }
        Ray3f r(emitterIts.p, -ray.d);
        return Le(r);
    }

    Color3f getDirectLightTo(const Scene *scene, const Ray3f &ray) const
    {
        Intersection its;

        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f); // No intersection, return black

        if (its.mesh != m_mesh)
            return Color3f(0.0f); // Not intersecting this emitter's mesh

        return getDirectLightTo(scene, ray, its);
    }

    Color3f getEmittedLightTo(const Scene *scene, const Intersection &originIts, const Intersection &emitterIts) const
    {
        Vector3f lightDir = originIts.p - emitterIts.p;
        float distance = lightDir.norm();
        Vector3f lightDirNormalized = lightDir / distance;

        Ray3f shadowRay(originIts.p, -lightDirNormalized);
        shadowRay.maxt = distance - Epsilon;

        if (scene->rayIntersect(shadowRay))
            return Color3f(0.0f); // Occluded, return black

        Color3f emittedLight = Le(shadowRay);
        float cosTheta1 = std::fabs(originIts.shFrame.n.dot(-lightDirNormalized));
        float cosTheta2 = std::fabs(emitterIts.shFrame.n.dot(lightDirNormalized));

        return emittedLight * cosTheta1 * cosTheta2 / (distance * distance);
    }

    /*
    Sample surface of emitter's mesh
    and return the reached light from sampled surface to the intersection
    */
    Color3f sampleEmittedLightTo(const Scene *scene, Sampler *sampler, const Intersection &originIts, Intersection &emitterIts) const
    {
        sampleSurface(sampler->next2D(), emitterIts);

        return getEmittedLightTo(scene, originIts, emitterIts);
    }

    virtual Color3f Le(const Ray3f &ray) const
    {
        // Default implementation returns black
        return Color3f(0.0f);
    }

private:
    Mesh *m_mesh = nullptr; ///< Pointer to the parent mesh (if any)
};

NORI_NAMESPACE_END
