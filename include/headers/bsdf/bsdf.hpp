
#ifndef PATH_TRACER_BSDF_HPP
#define PATH_TRACER_BSDF_HPP

#include "../math/vector2.hpp"
#include "bsdfSample.hpp"

namespace pathtracer{

    class Bsdf{


    public:

        Bsdf(){}


        // The cosine term is the normal dot wi, and since we
        // are in local coordinates, the normal is the z axis.
        static real cosineTheta(const Vector3& w) {
            return w.z();
        }    


        // The cosine term is the normal dot wi, and since we
        // are in local coordinates, the normal is the z axis.
        static real absCosineTheta(const Vector3& w) {
            return std::abs(w.z());
        }


        static float cosinePhiSineTheta(const Vector3& w) { 
            return w.x(); 
        }
        
        static float sinePhiSineTheta(const Vector3& w) { 
            return w.y(); 
        }
        
        
        // This returns the vector reflected around the normal,
        // which in local coordinates is always (0, 0, 1).
        static Vector3 reflect(const Vector3& w){
            return Vector3(-w.x(), -w.y(), w.z());
        }


        // Reflects around a given normal
        static Vector3 reflect(const Vector3 &w, const Vector3 &n) {
            return n * 2 * n.dot(w) - w;
        }

        
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