
#include "intersection/surfaceDifferentials.hpp"

namespace pathtracer{


    SurfaceDifferentials::SurfaceDifferentials() :
        m_dpdu(Vector3::ORIGIN),
        m_dpdv(Vector3::ORIGIN),
        m_dndu(Vector3::ORIGIN),
        m_dndv(Vector3::ORIGIN),
        m_d2pdu2(Vector3::ORIGIN),
        m_d2pdudv(Vector3::ORIGIN),
        m_d2pdv2(Vector3::ORIGIN),
        m_s(Vector3::ORIGIN) {}


    SurfaceDifferentials::SurfaceDifferentials(const Vector3& dpdu,
        const Vector3& dpdv, const Vector3& dndu,
        const Vector3& dndv, const Vector3& d2pdu2,
        const Vector3& d2pdudv, const Vector3& d2pdv2, const Vector3& s) :
        m_dpdu(dpdu), m_dpdv(dpdv), m_dndu(dndu), m_dndv(dndv),
        m_d2pdu2(d2pdu2), m_d2pdudv(d2pdudv), m_d2pdv2(d2pdv2), m_s(s) {}


    const Vector3& SurfaceDifferentials::dpdu() const {
        return m_dpdu;
    }


    const Vector3& SurfaceDifferentials::dpdv() const {
        return m_dpdv;
    }

    
    const Vector3& SurfaceDifferentials::dndu() const {
        return m_dndu;
    }


    const Vector3& SurfaceDifferentials::dndv() const {
        return m_dndv;
    }


    const Vector3& SurfaceDifferentials::d2pdu2() const {
        return m_d2pdu2;
    }


    const Vector3& SurfaceDifferentials::d2pdudv() const {
        return m_d2pdudv;
    }


    const Vector3& SurfaceDifferentials::d2pdv2() const {
        return m_d2pdv2;
    }


    const Vector3& SurfaceDifferentials::s() const {
        return m_s;
    }


    // The other tangent
    Vector3 SurfaceDifferentials::t(const Vector3& normal) const {
        return normal.cross(m_s);
    }

    
    void SurfaceDifferentials::makeOrthonormal() {
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

}