/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/common.h>
#include <nori/bsdf.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

/// Ideal dielectric BSDF
class Dielectric : public BSDF
{
public:
    Dielectric(const PropertyList &propList)
    {
        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);
    }

    Color3f eval(const BSDFQueryRecord &) const
    {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return Color3f(0.0f);
    }

    float pdf(const BSDFQueryRecord &) const
    {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return 0.0f;
    }

    Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const
    {
        bRec.measure = EDiscrete;
        float eta = bRec.eta = m_extIOR / m_intIOR;

        float cosThetaI = Frame::cosTheta(bRec.wi);
        float reflection_r = fresnel(cosThetaI, m_extIOR, m_intIOR);
        float transmission_r = 1.0f - reflection_r;

        if (sample.x() < reflection_r)
        {
            // Reflection
            bRec.wo = Vector3f(
                -bRec.wi.x(),
                -bRec.wi.y(),
                bRec.wi.z());
            return Color3f(reflection_r);
        }
        else
        {
            float sinPhi = Frame::sinPhi(bRec.wi);
            float cosPhi = Frame::cosPhi(bRec.wi);
            float sinThetaT = eta * std::sqrt(std::max(0.0f, 1.0f - cosThetaI * cosThetaI));
            float cosThetaT = std::sqrt(std::max(0.0f, 1.0f - sinThetaT * sinThetaT));

            bRec.wo = Vector3f(
                -cosPhi * sinThetaT,
                -sinPhi * sinThetaT,
                cosThetaT);
            return Color3f(transmission_r);
        }
    }

    std::string toString() const
    {
        return tfm::format(
            "Dielectric[\n"
            "  intIOR = %f,\n"
            "  extIOR = %f\n"
            "]",
            m_intIOR, m_extIOR);
    }

private:
    float m_intIOR, m_extIOR;
};

NORI_REGISTER_CLASS(Dielectric, "dielectric");
NORI_NAMESPACE_END
