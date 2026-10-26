
#ifndef PATH_TRACER_SPECULAR_MANIFOLD_BSDF_HPP
#define PATH_TRACER_SPECULAR_MANIFOLD_BSDF_HPP

#include "math/vector3.hpp"

namespace pathtracer{

    class Vector2;


    struct ManifoldNormal{
        Vector3 normal;

        // This a pdf in solid angles of sampling the normal in shading
        // space. Pdf should not change when we transform normal to world
        // coordinate since shading frame is assumed to be orthonormal.
        real pdf;
    };

    // This is a pure abstract class that specular BSDFs inherit from
    // in order to have the evaluate functions that return the weight
    // for the refraction and reflection cases.
    class SpecularManifoldBsdf{

        // TODO: ADD ADJOINT FLAG, AND POSSIBLY, WI FOR GLOSSY.

    public:

        // Given an incoming direction, returns the bsdf associated
        // with sampling the reflected direction. 
        // It does not make the random choice to reflect or refract,
        // but returns a weight whose pdf assumes reflection was chosen.
        // The wi direction is not sent in this eval, since only
        // one direction is valid anyway and it is assumed the caller
        // knows it.
        // The arguments are in local coordinates.
        // This computes weight, so includes cosine and pdf.
        // The wi is not used for perfectly specular surfaces
        // as it is assumed the wi is correct to start. For glossy
        // surfaces wi is use since we have a lobe of directions.
        virtual Vector3 evaluateReflection(const Vector3& wo,
            const Vector3& wi, const Vector2& uv) const = 0;


        // Same for refraction. Returns 0.0 if no refraction is possible.
        // The arguments are in local coordinates.
        virtual Vector3 evaluateRefraction(const Vector3& wo,
            const Vector3& wi, const Vector2& uv) const = 0;

        
        // Returns the IOR ratio between material going in and coming out 
        // at the specular surface.
        virtual real eta(const Vector3& wo, const Vector2& uv) const = 0;


        // Returns true if surface is rough (not pure delta)
        virtual bool isRough() const = 0;


        // Samples a manifold normal if the surface is rough. Returns
        // the shading normal otherwise.
        // NOTE: the normal is returned in local coordinates.
        virtual ManifoldNormal sampleManifoldNormal(const Vector2& uv) const = 0;


        virtual bool supportsReflection(const Vector2& uv) const = 0;


        virtual bool supportsRefraction(const Vector2& uv) const = 0;


    };

}

#endif