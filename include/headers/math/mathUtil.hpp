
#ifndef PATH_TRACER_MATH_UTIL_HPP
#define PATH_TRACER_MATH_UTIL_HPP

#include "constants.hpp"
#include "vector3.hpp"
#include "vector2.hpp"
#include "vector2i.hpp"

namespace pathtracer{


    class Util{

        public:  

        static int floor(real x){
            return static_cast<int>(x);
        }


        static int ceiling(real x){
            return static_cast<int>(x + 0.5);
        }


        static Vector2i floor(Vector2 uv){
            return Vector2i(floor(uv.x()), floor(uv.y()));
        }


        static Vector2i ceiling(Vector2 uv){
            return Vector2i(ceiling(uv.x()), ceiling(uv.y()));
        }


        static void swap(real& x, real& y){
            real temp = x;
            x = y;
            y = temp;
        }


        static real clamp(real value, real minVal, real maxVal){
            return (value < minVal) ? minVal : 
                (value > maxVal ? maxVal : value);
        }

    };


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
        // Note that this sampling pdf is the same for local area
        // and solid angle measures.
        static real pdf(const Vector3& point){
            return 0.25 * INV_PI;
        }

    };


    // Does the same for a Hemisphere 
    class SquareToHemisphereUniform{
    
    public: 
        
        // Transforms uv coordinates in a unit square to 3D
        // coordinates on a unit hemisphere uniformly.
        static Vector3 transform(const Vector2& uv){
            real theta = 2.0 * PI * uv.x();

            real z = uv.y();
            real r = sqrt(1.0 - z * z);

            return Vector3{
                r * cos(theta),
                r * sin(theta),
                z
            };
        }

        
        // The PDF is constant since it is uniform over the hemisphere 
        // area. It is half of the surface area of a sphere.
        // Note that this sampling pdf is the same for local area
        // and solid angle measures.
        static real pdf(const Vector3& point){
            return 0.5 * INV_PI;
        }

    };



    // Cosine weighted
    class SquareToHemisphereCosine{
    public:

        static Vector3 transform(const Vector2& uv){
            real r = sqrt(uv.x());
            real theta = 2.0 * PI * uv.y();

            real x = r * cos(theta);
            real y = r * sin(theta);
            real z = sqrt(1.0 - uv.x());

            return Vector3{
                x,
                y,
                z
            };
        }

        // The point is in shading coordinates space (normal is z axis)
        static real pdf(const Vector3& point){
            return point.z() * INV_PI;
        }

    };

}

#endif