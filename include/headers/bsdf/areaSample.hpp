
#ifndef PATH_TRACER_AREA_SAMPLE_HPP
#define PATH_TRACER_AREA_SAMPLE_HPP

#include "../core/transform.hpp"

namespace pathtracer{

    // This class represents a sample taken by randomly choosing
    // a point on the surface of a shape according to some probability
    // distribution, instead of using solid angles.
    class AreaSample{

    private:

        Vector3 m_position;

        // The pdf of having chosen the specific position m_position,
        // according to our sampling scheme.
        real m_pdf;

    public:

        AreaSample(const Vector3& position, real pdf) :
            m_position(position), m_pdf(pdf){}

        
        const Vector3& position() const { return m_position; }


        real pdf() const { return m_pdf; }

    };

}

#endif