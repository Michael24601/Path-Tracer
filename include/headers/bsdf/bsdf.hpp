
#ifndef PATH_TRACER_BSDF_HPP
#define PATH_TRACER_BSDF_HPP

#include "../math/vector2.hpp"
#include "bsdfSample.hpp"
#include "bsdfEvaluation.hpp"

namespace pathtracer{

    class Bsdf{

    protected:

        // The cosine term is the normal dot wi, and since we
        // are in local coordinates, the normal is the z axis.
        static real cosineTerm(const Vector3& wi) {
            return wi.z();
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


        // For the area formulation, we will instead pick a random
        // object, then sample a point on it. Both wi and the pdf
        // are already set, so we just use them to evaluate the
        // BSDF at this point, without randomly sampling a solid
        // angle wi. The wi direction and pdf are given as input
        // so they can be set automatically in the sample object.
        
        // If the BSDF is deterministic, like a perfect mirror that
        // only reflects light in wo from one specific direction wi,
        // the probability that the given wi matches that is 0,
        // so we return an invalid sample.
        virtual BsdfEvaluation evaluate(const Vector3& wo, 
            const Vector3& wi, const Vector2& uv) const = 0;
        

    };

}

#endif