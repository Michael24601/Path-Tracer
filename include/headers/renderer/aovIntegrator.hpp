
#ifndef PATH_TRACER_AOV_INTEGRATOR_HPP
#define PATH_TRACER_AOV_INTEGRATOR_HPP

#include "integrator.hpp"

namespace pathtracer{

    class AovIntegrator: public Integrator{

    private:

        enum class RenderVariable{
            ALBEDO,
            NORMAL
        };

        // Choice of variable to output, if we happen to intersect
        // an object.
        RenderVariable m_variable;

    public:

        AovIntegrator(RenderVariable variable) : m_variable{variable}{}

    
        // Uses whatever rendering technique it wants to render
        // the light received by the given shadow ray.
        Vector3 color(const Ray& ray, const Scene& scene) const override{

            Intersection it = scene.intersect(ray);
            
            // If background intersection
            if(!it){
                return Vector3::ORIGIN;
            }

            Vector3 color;

            // Otherwise
            switch(m_variable){
            case RenderVariable::ALBEDO:
                if(it.instance()->hasAlbedoTexture()){
                    color = it.instance()->albedoTexture()->sample(it.uv());
                }
                else{
                    // Red as a fallback
                    color = Vector3(1, 0, 0);
                }
                break;
            case RenderVariable::NORMAL:
                color = (it.shadingNormal() + Vector3(1.0)) * 0.5;
                break;
            default:
                color = Vector3::ORIGIN;
            }

            return color;
        }
     
    };

}

#endif