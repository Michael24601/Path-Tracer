
#include "light/directionalLight.hpp"
#include <cmath>
#include "intersection/surfaceDifferentials.hpp"
#include "light/lightSample.hpp"
#include "core/scene.hpp"
#include "math/vector2.hpp"
#include "core/random.hpp"
#include "math/constants.hpp"
#include "math/matrix3.hpp"
#include "logger.hpp"

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
        // it is 1 unit far away (if we compute origin - position we get
        // normalized wi)
        Vector3 position = origin + wi;

        return LightSample(wi, radiance, position, pdf, dist, this);
    }


    PhotonSample DirectionalLight::samplePhoton(const Scene& scene) const {

        // We don't have an origin for the directional light, so we
        // are forced to disperse photons from a disk that covers the
        // whole scene when photons are emitted following the direction
        // (its projection must cover the bounding box of the scene).
        const auto& box = scene.getBoundingBox();
        Vector3 center = (box.minCorner() + box.maxCorner()) * 0.5;
        real radius = (box.maxCorner() - box.minCorner()).length() * 0.5;

        Vector3 helper = std::abs(m_direction.z()) < 0.999
            ? Vector3(0, 0, 1)
            : Vector3(0, 1, 0);
        Vector3 t = helper.cross(m_direction).normalized();
        Vector3 b = m_direction.cross(t).normalized();

        // Then we just sample a random point on the disk
        Vector2 u = Random::next2D();
        real r = radius * std::sqrt(u.x());
        real phi = 2.0 * PI * u.y();
        Vector3 offset = t * (r * std::cos(phi)) + b * (r * std::sin(phi));

        // Moves the disk back along the direction a distance equal to the
        // radius, to ensure it is far enough away from the box.
        Vector3 origin = center + offset - m_direction * radius;

        // Position pdf is 1 / (pi R^2), so dividing by it is the same
        // as multiplying by the disk area.
        real diskArea = PI * radius * radius;
        Vector3 power = m_power * diskArea;

        return PhotonSample{power, origin, m_direction};
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
            Vector3(0.0), Vector3(0.0), Vector3(0.0)
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