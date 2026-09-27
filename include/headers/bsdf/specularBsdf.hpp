
#ifndef PATH_TRACER_SPECULAR_BSDF_HPP
#define PATH_TRACER_SPECULAR_BSDF_HPP

#include "bsdf.hpp"

namespace pathtracer{

    // This is a pure abstract class that specular BSDFs inherit from
    // in order to have the evaluate functions that return the weight
    // for the refraction and reflection cases.
    class SpecularBsdf: public Bsdf{


    public:

        // Has a delta distribution
        bool isSpecular() const override{
            return true;
        }


        // Given an incoming direction, returns the bsdf associated
        // with sampling the reflected direction. 
        // It does not make the random choice to reflect or refract,
        // but returns a weight whose pdf assumes reflection was chosen.
        // The wi direction is not sent in this eval, since only
        // one direction is valid anyway and it is assumed the caller
        // knows it.
        // The arguments are in local coordinates.
        // This computes bsdf, so no cosine or pdf.
        virtual Vector3 evaluateReflection(const Vector3& wo,
            const Vector2& uv) const = 0;


        // Same for refraction. Returns 0.0 if no refraction is possible.
        // The arguments are in local coordinates.
        virtual Vector3 evaluateRefraction(const Vector3& wo,
            const Vector2& uv) const = 0;

        
        // Returns the IOR ratio between material going in and coming out 
        // at the specular surface.
        virtual real eta(const Vector3& wo, const Vector2& uv) const = 0;

    };

}

#endif