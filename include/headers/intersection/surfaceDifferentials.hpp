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

        Vector3 m_d2pdu2;
        Vector3 m_d2pdudv;
        Vector3 m_d2pdv2;

        // It also stores the tangent s computed from dndu
        Vector3 m_s;

    public:

        SurfaceDifferentials() :
            m_dpdu(Vector3::ORIGIN),
            m_dpdv(Vector3::ORIGIN),
            m_dndu(Vector3::ORIGIN),
            m_dndv(Vector3::ORIGIN),
            m_d2pdu2(Vector3::ORIGIN),
            m_d2pdudv(Vector3::ORIGIN),
            m_d2pdv2(Vector3::ORIGIN),
            m_s(Vector3::ORIGIN) {}

        SurfaceDifferentials(
            const Vector3& dpdu,
            const Vector3& dpdv,
            const Vector3& dndu,
            const Vector3& dndv,
            const Vector3& d2pdu2,
            const Vector3& d2pdudv,
            const Vector3& d2pdv2,
            const Vector3& s
        ) :
            m_dpdu(dpdu),
            m_dpdv(dpdv),
            m_dndu(dndu),
            m_dndv(dndv),
            m_d2pdu2(d2pdu2),
            m_d2pdudv(d2pdudv),
            m_d2pdv2(d2pdv2),
            m_s(s) {}

        const Vector3& dpdu() const { return m_dpdu; }
        const Vector3& dpdv() const { return m_dpdv; }
        const Vector3& dndu() const { return m_dndu; }
        const Vector3& dndv() const { return m_dndv; }

        const Vector3& d2pdu2() const { return m_d2pdu2; }
        const Vector3& d2pdudv() const { return m_d2pdudv; }
        const Vector3& d2pdv2() const { return m_d2pdv2; }

        const Vector3& s() const { return m_s; }

        // The other tangent
        Vector3 t(const Vector3& normal) const {
            return normal.cross(m_s);
        }


        void makeOrthonormal() {
            real invNorm = 1.0 / m_dpdu.length();

            m_dpdu = m_dpdu * invNorm;
            m_dndu = m_dndu * invNorm;

            real dp = m_dpdu.dot(m_dpdv);

            Vector3 dpdvTmp = m_dpdv - m_dpdu * dp;
            Vector3 dndvTmp = m_dndv - m_dndu * dp;

            invNorm = 1.0 / dpdvTmp.length();

            m_dpdv = dpdvTmp * invNorm;
            m_dndv = dndvTmp * invNorm;
        }

    };

}

#endif