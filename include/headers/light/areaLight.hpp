

#ifndef PATH_TRACER_AREA_LIGHT_HPP
#define PATH_TRACER_AREA_LIGHT_HPP

#include "light.hpp"
#include "../core/instance.hpp"
#include "../intersection/areaSample.hpp"

namespace pathtracer{

    class AreaLight: public Light{

    private:

        // The instance that emits light the area light refers to
        Instance* m_instance;
        
   
    public:

        AreaLight(Instance* instance) : m_instance{instance}{

            // Ensures the instance is emissive
            assert((instance->emission()) && "Object is not emissive");

            // Then we tell the instance it is a light
            instance->setLight(this);
        }


        // Calculates in global coordinates.
        LightSample sample(const Vector3& origin) const override{

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
                solidAnglePdf, dist);

            return sample;
        }

        
        
        LightSample evaluateLightSample(const Vector3& origin,
            const Vector3& point) const override{
            
            // We just evaluate the area sample of having chosen
            // this particular point.
            AreaSample s = m_instance->evaluateAreaSample(point);
            Vector3 wi = (s.position() - origin).normalized();

            // The light sample expects the pdf in solid angles
            real cosineY = std::abs(s.shadingNormal().dot(-wi));
            real dist = (s.position() - origin).length();
            real solidAnglePdf =  s.pdf() * (dist * dist) / cosineY;

            // The radiance is just the emission at this point
            Vector3 radiance = s.evaluateEmission(-wi);

            return LightSample(wi, radiance, s.position(), 
                solidAnglePdf, dist);
        }


        bool isIntersectable() const override{
            return m_instance->inScene();
        }
    
    };

}

#endif