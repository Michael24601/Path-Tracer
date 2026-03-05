
#ifndef PATH_TRACER_BSDF_EVALUATION_HPP
#define PATH_TRACER_BSDF_EVALUATION_HPP

#include "../core/ray.hpp"

namespace pathtracer{

    class BsdfEvaluation{

        // The bsdf term fr(x, wi, wo)
        Vector3 m_bsdf;

        bool m_invalid;

    public:

        static BsdfEvaluation INVALID;

        
        // The evluation is done without sampling wi, so we only
        // return the bsdf.
        BsdfEvaluation(const Vector3& bsdf, bool invalid):
            m_bsdf{bsdf}, m_invalid{invalid}{}


        const Vector3& bsdf() const { return m_bsdf; }


        bool isInvalid() const {
            return m_invalid;
        }
        

    };

}

#endif