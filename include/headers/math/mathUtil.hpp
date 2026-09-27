#ifndef PATH_TRACER_MATH_UTIL_HPP
#define PATH_TRACER_MATH_UTIL_HPP

#include "config.hpp"

namespace pathtracer{


    class Vector3;
    class Vector2;
    class Vector2i;


    namespace Util{

        float russianRoulette(const Vector3& c, float max = 0.75f);

        int floor(real x);

        int ceiling(real x);

        Vector2i floor(Vector2 uv);

        Vector2i ceiling(Vector2 uv);

        void swap(real& x, real& y);

        real clamp(real value, real minVal, real maxVal);

    }


    namespace ShadingSpace{

        // The cosine term is the normal dot wi, and since we
        // are in local coordinates, the normal is the z axis.
        real cosineTheta(const Vector3& w);

        // The cosine term is the normal dot wi, and since we
        // are in local coordinates, the normal is the z axis.
        real absCosineTheta(const Vector3& w);

        float cosinePhiSineTheta(const Vector3& w);

        float sinePhiSineTheta(const Vector3& w);

        // This returns the vector reflected around the normal,
        // which in local coordinates is always (0, 0, 1).
        Vector3 reflect(const Vector3& w);

        // Reflects around a given normal
        Vector3 reflect(const Vector3& w, const Vector3& n);

        // Refracts a vector given ior and normal
        Vector3 refract(const Vector3& w, const Vector3& n, real eta);

    }


    namespace Barycentric{

        // Interpolates 3D vector
        Vector3 interpolate(const Vector3& v0, const Vector3& v1,
            const Vector3& v2, const Vector2& uv);

        // Interpolates a 2D vector
        Vector2 interpolate(const Vector2& v0, const Vector2& v1,
            const Vector2& v2, const Vector2& uv);

    }


    // Converts a 2D coordinate between (0, 0) and (1, 1) to
    // a coordinate on a sphere. This is not the standard parametric 
    // mapping, as it ensures the sampling remains uniform on the 
    // sphere as well.
    // This can be checked by multiplying the square pdf by the change 
    // of measure term, which gives us the final sphere pdf.
    // In fact, the mapping is precisely designed using inverse
    // transform sampling in order to cancel out the variable part 
    // of the pdf, leaving a constant term.
    namespace SquareToSphereUniform{

        // Transforms uv coordinates in a unit square to 3D
        // coordinates on a unit sphere uniformly.
        Vector3 transform(const Vector2& uv);

        // Return spherical coordinates
        Vector2 inverse(const Vector3& d);

        // The PDF is constant since it is uniform over the surface area.
        // Note that this sampling pdf is the same for local area
        // and solid angle measures.
        real pdf(const Vector3& point);

    }


    // Does the same for a Hemisphere
    namespace SquareToHemisphereUniform{

        // Transforms uv coordinates in a unit square to 3D
        // coordinates on a unit hemisphere uniformly.
        Vector3 transform(const Vector2& uv);

        // Transforms 3D coordinates on a unit hemisphere.
        Vector2 inverse(const Vector3& dir);

        // The PDF is constant since it is uniform over the hemisphere 
        // area. It is half of the surface area of a sphere.
        // Note that this sampling pdf is the same for local area
        // and solid angle measures.
        real pdf(const Vector3& point);

    }


    // Cosine weighted
    namespace SquareToHemisphereCosine{

        Vector3 transform(const Vector2& uv);

        // The point is in shading coordinates space (normal is z axis)
        real pdf(const Vector3& point);

    }

}

#endif