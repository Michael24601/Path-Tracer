

#ifndef PATH_TRACER_POINT_LIGHT_HPP
#define PATH_TRACER_POINT_LIGHT_HPP

#include "light.hpp"
#include "../math/vector3.hpp"
#include "../math/constants.hpp"

namespace pathtracer{


    class PointLight: public Light{

    private:

        // The light's position
        Vector3 m_position;

        // The light's color
        Vector3 m_power;
   
    public:

        PointLight(const Vector3& position, const Vector3& power) : 
            Light(LightType::NON_INTERSECTABLE),
            m_position{position}, m_power{power}{}


        // Calculates in global coordinates.
        LightSample sample(const Vector3& origin) const override{

            // There is only one point, so we can't sample points,
            // we have to choose the one. So the pdf is 1.0.
            real pdf = 1.0;

            // The power radiates in a sphere, so.
            real dist = (origin - m_position).length();
            real distSquared = dist * dist;
            Vector3 radiance = m_power * (1.0 / (4 * PI * distSquared));

            Vector3 wi = (m_position - origin).normalized();

            return LightSample(wi, radiance, m_position, pdf, dist);
        }


        LightSample evaluateLightSample(const Vector3& origin,
            const Vector3& point) const override{
            
            // Non intersectable light
            return LightSample::INVALID;
        }

    
    };

}

#endif