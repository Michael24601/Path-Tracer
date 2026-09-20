
#ifndef PATH_TRACER_SURFACE_DIFFERENTIALS_HPP
#define PATH_TRACER_SURFACE_DIFFERENTIALS_HPP

#include "../math/vector3.hpp"

namespace pathtracer{

    // Reference = https://pbr-book.org/4ed/Geometry_and_Transformations/Interactions
    class SurfaceDifferentials{

    private:

        Vector3 m_dpdu;
        Vector3 m_dpdv;
        Vector3 m_dndu;
        Vector3 m_dndv;

    public:

        SurfaceDifferentials() : m_dpdu(Vector3::ORIGIN), 
            m_dpdv(Vector3::ORIGIN), m_dndu(Vector3::ORIGIN), 
            m_dndv(Vector3::ORIGIN) {}

        SurfaceDifferentials(const Vector3& dpdu, const Vector3& dpdv,
            const Vector3& dndu, const Vector3& dndv) : 
            m_dpdu(dpdu), m_dpdv(dpdv), m_dndu(dndu), m_dndv(dndv) {}

        const Vector3& dpdu() const { return m_dpdu; }
        const Vector3& dpdv() const { return m_dpdv; }
        const Vector3& dndu() const { return m_dndu; }
        const Vector3& dndv() const { return m_dndv; }

    };
}

#endif