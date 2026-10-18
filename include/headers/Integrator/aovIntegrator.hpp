#ifndef PATH_TRACER_AOV_INTEGRATOR_HPP
#define PATH_TRACER_AOV_INTEGRATOR_HPP

#include "integrator.hpp"

namespace pathtracer{

    class AovIntegrator: public Integrator{

    public:

        enum class RenderVariable{
            ALBEDO,
            NORMAL,
            DIRECTION,
            DPDU,
            DPDV,
            DNDU,
            DNDV,
            DSDU,
            DSDV,
            DTDU,
            DTDV,
            S
        };

    private:

        // Choice of variable to output, if we happen to intersect
        // an object.
        RenderVariable m_variable;

    public:

        AovIntegrator(RenderVariable variable);

        // Uses whatever rendering technique it wants to render
        // the light received by the given shadow ray.
        Vector3 color(const Ray& ray, const Scene& scene) override;


        std::string toString() const override{

            std::string name;

            switch (m_variable){
                case RenderVariable::ALBEDO: name = "ALBEDO"; break;
                case RenderVariable::NORMAL: name = "NORMAL"; break;
                case RenderVariable::DIRECTION: name = "DIRECTION"; break;
                case RenderVariable::DPDU: name = "DPDU"; break;
                case RenderVariable::DPDV: name = "DPDV"; break;
                case RenderVariable::DNDU: name = "DNDU"; break;
                case RenderVariable::DNDV: name = "DNDV"; break;
                case RenderVariable::DSDU: name = "DSDU"; break;
                case RenderVariable::DSDV: name = "DSDV"; break;
                case RenderVariable::DTDU: name = "DTDU"; break;
                case RenderVariable::DTDV: name = "DTDV"; break;
                case RenderVariable::S: name = "S"; break;
                default: name = "";
            }

            return "AOV VARIABLE=" + name;
        }
    };

}

#endif