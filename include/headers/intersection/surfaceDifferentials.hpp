#ifndef PATH_TRACER_SURFACE_DIFFERENTIALS_HPP
#define PATH_TRACER_SURFACE_DIFFERENTIALS_HPP

#include "math/vector3.hpp"

namespace pathtracer{

    
    class SurfaceDifferentials{

    private:

        Vector3 m_dpdu;
        Vector3 m_dpdv;
        Vector3 m_dndu;
        Vector3 m_dndv;

        Vector3 m_d2pdu2;
        Vector3 m_d2pdudv;
        Vector3 m_d2pdv2;

        // It also stores the tangent s computed from dndu
        Vector3 m_s;

    public:

        SurfaceDifferentials();

        SurfaceDifferentials(const Vector3& dpdu, const Vector3& dpdv, 
            const Vector3& dndu, const Vector3& dndv,
            const Vector3& d2pdu2, const Vector3& d2pdudv,
            const Vector3& d2pdv2, const Vector3& s);

        const Vector3& dpdu() const;
        const Vector3& dpdv() const;
        const Vector3& dndu() const;
        const Vector3& dndv() const;

        const Vector3& d2pdu2() const;
        const Vector3& d2pdudv() const;
        const Vector3& d2pdv2() const;

        const Vector3& s() const;

        // The other tangent
        Vector3 t(const Vector3& normal) const;

        void makeOrthonormal();

    };

}

#endif