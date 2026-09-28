
#include "light/directionalLight.hpp"
#include <cmath>
#include "intersection/surfaceDifferentials.hpp"
#include "light/lightSample.hpp"
#include "math/constants.hpp"

namespace pathtracer{


    DirectionalLight::DirectionalLight(const Vector3& direction, 
        const Vector3& power) : m_direction{direction.normalized()}, 
        m_power{power} {}

        
    // Calculates in global coordinates.
    LightSample DirectionalLight::sample(const Vector3& origin) const {

        // There is only one direction, so we can't sample at random,
        // we have to choose the one. So the pdf is 1.0.
        real pdf = 1.0;
        // The source is infinitely far, so
        real dist = REAL_INFINITY;
        Vector3 radiance = m_power;
        // The direction is opposite as wi starts at the origin
        Vector3 wi = -m_direction;
        // The point doesn't exist but for the sake of the sample
        // it is infiniteky far away
        Vector3 position = origin + wi * dist;

        return LightSample(wi, radiance, position, pdf, dist, this);
    }


    LightSample DirectionalLight::evaluateLightSample(
        const Vector3& origin,
        const SurfaceSample& point) const {

        // Non intersectable light
        return LightSample::INVALID;
    }


    SurfaceDifferentials DirectionalLight::computeDifferentials(
        const LightSample& s) const {

        Vector3 n = s.wi();
        Vector3 tangent;

        // We construct a frame orthogonal to the direction
        if (std::abs(n.x()) > std::abs(n.z())) {
            tangent = Vector3(-n.y(), n.x(), 0.0).normalized();
        }
        else {
            tangent = Vector3(0.0, -n.z(), n.y()).normalized();
        }

        Vector3 bitangent = n.cross(tangent);

        // Since the frame is flat, dn, ds, dt... are all 0
        return SurfaceDifferentials(
            tangent, bitangent,
            Vector3(0.0), Vector3(0.0),
            Vector3(0.0), Vector3(0.0), Vector3(0.0),
            tangent
        );
    }


    bool DirectionalLight::hasArea() const {
        return false;
    }


    bool DirectionalLight::isDirectional() const{
        return true;
    }


    bool DirectionalLight::isPoint() const{
        return false;
    }
    

    bool DirectionalLight::isIntersectable() const {
        return false;
    }

}