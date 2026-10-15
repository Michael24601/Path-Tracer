
#include "light/areaLight.hpp"
#include <cassert>
#include "core/instance.hpp"
#include "intersection/areaSample.hpp"
#include "light/lightSample.hpp"
#include "math/mathUtil.hpp"
#include "intersection/surfaceDifferentials.hpp"
#include "shapes/shape.hpp"
#include "core/random.hpp"
#include "math/vector2.hpp"

namespace pathtracer{
    

    AreaLight::AreaLight(Instance* instance) :
        m_instance{instance} {

        // Ensures the instance is emissive
        assert((instance->emission()) && "Object is not emissive");

        // Then we tell the instance it is a light
        instance->setLight(this);
    }


    // Calculates in global coordinates.
    LightSample AreaLight::sample(const Vector3& origin) const {

        // We just sample a random point on an instance,
        // which returns a sample with a pdf in world coordinates,
        // in the area measure.
        AreaSample s = m_instance->sampleArea();

        Vector3 wi = (s.position() - origin).normalized();

        // The light sample expects the pdf in solid angles
        real cosineY = s.shadingNormal().dot(-wi);

        // Guarding against the backface
        if (cosineY <= 0) return LightSample::INVALID;

        real dist = (s.position() - origin).length();
        real solidAnglePdf = s.pdf() * (dist * dist) / cosineY;

        // The radiance is just the emission at this point
        // (Here wo is the opposite of wi).
        Vector3 radiance = s.evaluateEmission(-wi);

        LightSample sample(wi, radiance, s.position(),
            solidAnglePdf, dist, this, cosineY, s.triangleIndex());

        return sample;
    }


    PhotonSample AreaLight::samplePhoton() const{

        // Samples a random point
        AreaSample s = m_instance->sampleArea();

        // Samples a random direction locally in upper hemisphere
        Vector3 localWo = SquareToHemisphereUniform::transform(Random::next2D());
        Vector3 wo = s.shadingFrame().transformDirection(localWo);

        // The light sample expects the pdf in solid angles
        real cosineY = std::max(0.0, s.shadingNormal().dot(wo));
        real solidAnglePdf = s.pdf() / cosineY;

        real directionPdf = SquareToHemisphereUniform::pdf(localWo);

        // The radiance is just the emission at this point
        // (Here wo is the opposite of wi).
        Vector3 radiance = s.evaluateEmission(wo);
        radiance = radiance / (directionPdf * solidAnglePdf);

        return PhotonSample{radiance, s.position(), wo};

    }


    LightSample AreaLight::evaluateLightSample(
        const Vector3& origin,
        const SurfaceSample& surPoint) const {

        // We just evaluate the area sample of having chosen
        // this particular point.
        AreaSample s = m_instance->evaluateAreaSample(surPoint);

        Vector3 wi = (s.position() - origin).normalized();

        // The light sample expects the pdf in solid angles
        real cosineY = std::abs(s.shadingNormal().dot(-wi));
        real dist = (s.position() - origin).length();
        real solidAnglePdf = s.pdf() * (dist * dist) / cosineY;

        // The radiance is just the emission at this point
        Vector3 radiance = s.evaluateEmission(-wi);

        return LightSample(wi, radiance, s.position(),
            solidAnglePdf, dist, this, cosineY, s.triangleIndex());
    }


    SurfaceDifferentials AreaLight::computeDifferentials(
        const LightSample& s) const {

        // First we get the area sample for s
        SurfaceSample sample(s.position(), s.triangleIndex());
        AreaSample lightSample =
            m_instance->evaluateAreaSample(sample);

        return m_instance->computeDifferentials(lightSample);
    }


    bool AreaLight::hasArea() const {
        return true;
    }


    bool AreaLight::isIntersectable() const {
        return m_instance->inScene();
    }


    bool AreaLight::isDirectional() const{
        return false;
    }


    bool AreaLight::isPoint() const{
        return false;
    }

}