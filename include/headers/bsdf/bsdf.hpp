
#ifndef PATH_TRACER_BSDF_HPP
#define PATH_TRACER_BSDF_HPP

#include "../math/vector2.hpp"
#include "bsdfSample.hpp"

namespace pathtracer{

    class Bsdf{

    protected:

        // The cosine term is the normal dot wi, and since we
        // are in local coordinates, the normal is the z axis.
        static real cosineTerm(const Vector3& wi) {
            return wi.z();
        }    
        
        
        // This returns the vector reflected around the normal,
        // which in local coordinates is always (0, 0, 1).
        static Vector3 reflect(const Vector3& wo){
            return Vector3(-wo.x(), -wo.y(), wo.z());
        }


    public:

        Bsdf(){}

        
        // In general, we can either sample a point directly
        // on the surface of an object, after choosing said object,
        // or we can sample using solid angles. We use the latter
        // for a BSDF since some BSDFs require a specific angle
        // to be sampled from with respect to the exitant direction
        // wo.
        // Note that this in the shading frame coordinates, 
        // and wo is expected to be transformed already.
        virtual BsdfSample sample(const Vector3& wo, const Vector2& uv) 
            const = 0;


        // This is the reverse query, we pick the wi ourselves,
        // So wi is already chosen.
        // This evaluate function takes the given wi and returns the
        // pdf of having sampled said point, along with the cosine
        // term and bsdf of the given wi.
        // This is useful for MIS, where we need to find the pdf
        // of having sampled some given wi.
        // It is also useful for the area formulation or NEE, where we
        // sample a point, which gives us a wi, and we need to
        // evaluate the bsdf value without sampling.
        virtual BsdfSample evaluate(const Vector3& wo, 
            const Vector3& wi, const Vector2& uv) const = 0;
        

    };

}

#endif