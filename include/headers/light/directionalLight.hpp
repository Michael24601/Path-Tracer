
#ifndef PATH_TRACER_DIRECTIONAL_LIGHT_HPP
#define PATH_TRACER_DIRECTIONAL_LIGHT_HPP

#include "light.hpp"
#include "math/vector3.hpp"

namespace pathtracer{

    class DirectionalLight: public Light{

    private:

        // The light's direction, assumed to be normalized
        Vector3 m_direction;

        // The light's color
        Vector3 m_power;

    public:

        DirectionalLight(const Vector3& direction, const Vector3& power);

        // Calculates in global coordinates.
        LightSample sample(const Vector3& origin) const override;

        LightSample evaluateLightSample(const Vector3& origin,
            const SurfaceSample& point) const override;

        SurfaceDifferentials computeDifferentials(const LightSample& s) 
            const override;

        bool hasArea() const override;

        bool isDirectional() const override;

        bool isPoint() const override;

        bool isIntersectable() const override;

    };

}

#endif