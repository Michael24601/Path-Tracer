
#include "intersection/intersection.hpp"
#include "math/constants.hpp"
#include <limits>

namespace pathtracer{


    Intersection Intersection::NO_HIT = Intersection();


    Intersection::Intersection() : m_t{REAL_INFINITY},
        SurfacePoint(Vector3(0.0), Vector3(0.0), Vector3(0.0),
        Vector3(0.0), Vector2(0.0), nullptr) {}


    Intersection::Intersection(real t, const Vector3& position,
        const Vector3& geometryNormal, const Vector3& shadingNormal,
        const Vector3& tangent, const Vector2& uv, const Instance* instance) :
        m_t{t}, SurfacePoint(position, geometryNormal, shadingNormal,
            tangent, uv, instance) {}


    Intersection::Intersection(
        real t, const SurfacePoint& sp) :
        m_t{t},
        SurfacePoint(sp) {}


    const real Intersection::t() const {
        return m_t;
    }


    void Intersection::setT(real t) {
        m_t = t;
    }


    // Returns false if not a hit
    Intersection::operator bool() const {
        return m_t < std::numeric_limits<real>::infinity();
    }

}