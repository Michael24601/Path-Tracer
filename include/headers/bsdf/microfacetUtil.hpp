/**
 * Functions for dealing with microfacet distributions.
 * Based on the microfacet distribution implementation provided by
 * the Lightwave framework.
 */ 

#ifndef PATH_TRACER_MICROFACET_UTIL_HPP
#define PATH_TRACER_MICROFACET_UTIL_HPP

#include "config.hpp"

namespace pathtracer{

    class Vector3;
    class Vector2;

    namespace Microfacet{

        real smithG1(real alpha, const Vector3& wh, const Vector3& w);

        real evaluateGGX(real alpha, const Vector3& wh);

        Vector3 sampleGGXVNDF(real alpha, const Vector3& wo, 
            const Vector2& rnd);

        real pdfGGXVNDF(real alpha, const Vector3& wh, const Vector3& wo);

    }
}

#endif