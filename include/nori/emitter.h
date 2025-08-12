/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#pragma once

#include <nori/object.h>
#include <nori/mesh.h>
#include <nori/sampler.h>
#include <nori/scene.h>
#include <nori/dpdf.h>

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

    void setParent(NoriObject *parent) override;

    const Mesh *getMesh() const { return m_mesh; }

    void sampleSurface(const Point2f &sample, Intersection &its) const;

    bool sampleEmitter(const Scene *scene, const Point2f &sample, const Intersection &emitteeIts, Intersection &emitterIts, float &pdf) const;

    bool sampleEmitter(const Scene *scene, const Point2f &sample, const Intersection &emitteeIts, Intersection &emitterIts, float &pdf, float &cosTheta) const;

    float pdf(const Scene *scene, const Intersection &emitteeIts, const Intersection &emitterIts) const;

    float pdf(const Scene *scene, const Intersection &emitteeIts, const Intersection &emitterIts, float &cosTheta) const;

    virtual Color3f Le(const Ray3f &ray = Ray3f()) const;

    static const std::vector<Emitter *> &getEmitters()
    {
        return m_emitters;
    }

private:
    Mesh *m_mesh = nullptr;                   ///< Pointer to the parent mesh (if any)
    int m_surfaceIndex;                       ///< Index of the surface in the PDF
    static DiscretePDF m_surfacePdf;          ///< Discrete PDF for surface sampling
    static std::vector<Emitter *> m_emitters; ///< All emitters in the scene
    static bool m_bSampleSurfaceInitialized;  ///< Has the surface PDF been initialized?
};

NORI_NAMESPACE_END