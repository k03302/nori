#include <nori/emitter.h>

NORI_NAMESPACE_BEGIN

class Area : public Emitter
{
public:
    Area(const PropertyList &props)
    {
        m_radiance = props.getColor("radiance", Color3f(1.0f));
    }

    std::string toString() const
    {
        return "Area[]";
    }

    Color3f Le(const Ray3f &ray) const override
    {
        return m_radiance;
    }

private:
    Color3f m_radiance;
};

NORI_REGISTER_CLASS(Area, "area");
NORI_NAMESPACE_END