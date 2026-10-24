
#include "light/pointLight.hpp"
#include <cmath>
#include "intersection/surfaceDifferentials.hpp"
#include "light/lightSample.hpp"
#include "math/mathUtil.hpp"
#include "math/constants.hpp"
#include "core/random.hpp"
#include "math/vector2.hpp"

namespace pathtracer{


    PointLight::PointLight(const Vector3& position, const Vector3& power) :
        m_position{position}, m_power{power} {}

        
    // Calculates in global coordinates.
    LightSample PointLight::sample(const Vector3& origin) const {

        // There is only one point, so we can't sample points,
        // we have to choose the one. So the pdf is 1.0.
        real pdf = 1.0;

        // The power radiates in a sphere, so.
        real dist = (m_position - origin).length();
        real distSquared = dist * dist;
        Vector3 radiance =
            m_power * (1.0 / (4 * PI * distSquared));

        Vector3 wi =
            (m_position - origin).normalized();

        return LightSample(wi, radiance, m_position, pdf, dist, this);
    }


    PhotonSample PointLight::samplePhoton() const{
        // The point on the light is always the same, the only thing
        // to sample is the direction.
        
        // Samples a random direction locally in upper hemisphere
        Vector3 wo = SquareToSphereUniform::transform(Random::next2D());
        real directionPdf = SquareToHemisphereUniform::pdf(wo);

        // Notice that power has nothing to do with distance.
        // In photon mapping, photons arrive with the same power no
        // matter the distance.
        Vector3 power = m_power / directionPdf;

        return PhotonSample{power, m_position, wo};
    }


    LightSample PointLight::evaluateLightSample(
        const Vector3& origin,
        const SurfaceSample& point) const {

        // Non intersectable light
        return LightSample::INVALID;
    }


    SurfaceDifferentials PointLight::computeDifferentials(
        const LightSample& s) const {

        Vector3 n = s.wi();
        Vector3 tangent;

        // Flast orthogonal frame to wi
        if (std::abs(n.x()) > std::abs(n.z())){
            tangent = Vector3(-n.y(), n.x(), 0.0).normalized();
        }
        else{
            tangent = Vector3(0.0, -n.z(), n.y()).normalized();
        }

        Vector3 bitangent = n.cross(tangent);

        // Since the frame is flat, dn, ds, dt... are all 0
        return SurfaceDifferentials(
            tangent, bitangent,
            Vector3(0.0), Vector3(0.0),
            Vector3(0.0), Vector3(0.0), Vector3(0.0)
        );
    }


    bool PointLight::hasArea() const {
        return false;
    }


    bool PointLight::isDirectional() const{
        return false;
    }


    bool PointLight::isPoint() const{
        return true;
    }
    

    bool PointLight::isIntersectable() const {
        return false;
    }

}