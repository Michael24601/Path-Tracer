
#ifndef PATH_TRACER_MATH_UTIL_HPP
#define PATH_TRACER_MATH_UTIL_HPP

#include "config.hpp"
#include "constants.hpp"
#include "vector3.hpp"
#include "vector2.hpp"

namespace pathtracer{


    class Barycentric{

    public:

        // Interpolates 3D vector
        static Vector3 interpolate(const Vector3& v0, 
            const Vector3& v1, const Vector3& v2, const Vector2& uv){

            real u = uv.x();
            real v = uv.y();
            real w = 1.0 - u - v;

            return v0 * w + v1 * u + v2 * v;
        }


        // Interpolates a 2D vector
        static Vector2 interpolate(const Vector2& v0, 
            const Vector2& v1, const Vector2& v2, const Vector2& uv){

            real u = uv.x();
            real v = uv.y();
            real w = 1.0 - u - v;

            return v0 * w + v1 * u + v2 * v;
        }

    };


    // Converts a 2D coordinate between (0, 0) and (1, 1) to
    // a coordinate on a sphere. This is not the standard parametric 
    // mapping, as it ensures the sampling remains uniform on the 
    // sphere as well.
    // This can be checked by multiplying the square pdf by the change 
    // of measure term, which gives us the final sphere pdf.
    // In fact, the mapping is precisely designed using inverse
    // transform sampling in order to cancel out the variable part 
    // of the pdf, leaving a constant term.
    class SquareToSphereUniform{
    
    public: 
        
        // Transforms uv coordinates in a unit square to 3D
        // coordinates on a unit sphere uniformly.
        static Vector3 transform(const Vector2& uv){
            real theta = 2.0 * PI * uv.x();
            real phi   = acos(1.0 - 2.0 * uv.y());

            real sinPhi = sin(phi);

            return Vector3{
                sinPhi * cos(theta),
                sinPhi * sin(theta),
                cos(phi)
            };
        }


        // The PDF is constant since it is uniform over the surface area.
        static real pdf(const Vector3& point){
            return 0.25 * INV_PI;
        }

    };

}

#endif