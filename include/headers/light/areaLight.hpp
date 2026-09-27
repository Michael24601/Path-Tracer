
#ifndef PATH_TRACER_AREA_LIGHT_HPP
#define PATH_TRACER_AREA_LIGHT_HPP

#include "light.hpp"

namespace pathtracer{

    class Instance;

    class AreaLight: public Light{

    private:

        // The instance that emits light the area light refers to
        Instance* m_instance;

    public:

        AreaLight(Instance* instance);

        // Calculates in global coordinates.
        LightSample sample(const Vector3& origin) const override;

        LightSample evaluateLightSample(const Vector3& origin,
            const SurfaceSample& surPoint) const override;

        SurfaceDifferentials computeDifferentials(
            const LightSample& s) const override;

        bool hasArea() const override;

        bool isIntersectable() const override;

    };

}

#endif