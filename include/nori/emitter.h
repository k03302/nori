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

    void sampleSurface(const Point2f &sample, Point3f &p, Vector3f &n) const
    {
        if (m_mesh == nullptr)
            throw NoriException("Emitter::sampleSurface() called on an emitter without a parent mesh!");

        m_mesh->sampleSurface(sample, p, n);
    }

    /*
    Sample surface of emitter's mesh
    and return the reached light from sampled surface to the intersection
    */
    Color3f sampleEmittedLightTo(const Scene *scene, Sampler *sampler, const Intersection &its) const
    {
        Point3f p;
        Vector3f n;
        sampleSurface(sampler->next2D(), p, n);

        Vector3f lightDir = p - its.p;
        float distance = lightDir.norm();
        Vector3f lightDirNormalized = lightDir.normalized();

        Ray3f shadowRay(its.p, lightDirNormalized);
        shadowRay.maxt = distance - Epsilon;

        if (scene->rayIntersect(shadowRay))
            return Color3f(0.0f); // Occluded, return black

        Color3f emittedLight = Le(shadowRay);
        float cosTheta1 = std::max(0.0f, its.shFrame.n.dot(-lightDirNormalized));
        float cosTheta2 = std::max(0.0f, n.dot(lightDirNormalized));

        return emittedLight * cosTheta1 * cosTheta2 / (distance * distance);
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
