
#ifndef PATH_TRACER_AREA_SAMPLE_HPP
#define PATH_TRACER_AREA_SAMPLE_HPP

#include "surfacePoint.hpp"

namespace pathtracer{

    // This class represents a sample taken by randomly choosing
    // a point on the surface of a shape according to some probability
    // distribution, instead of using solid angles.
    class AreaSample: public SurfacePoint{

    private:

        // The pdf of having chosen the specific position m_position,
        // according to our sampling scheme.
        real m_pdf;

    public:

        AreaSample(const Vector3& position, 
            const Vector3& geometryNormal, const Vector3& shadingNormal,
            const Vector3& tangent, const Vector2& uv, 
            const Instance* instance, real pdf) : 
            m_pdf(pdf), SurfacePoint(position, geometryNormal, 
                shadingNormal, tangent, uv, instance){}


        AreaSample(const SurfacePoint& sp, real pdf) : 
            m_pdf(pdf), SurfacePoint(sp){}


        real pdf() const { return m_pdf; }

    };

}

#endif