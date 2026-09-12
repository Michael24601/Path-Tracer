
#ifndef PATH_TRACER_SHAPE_HPP
#define PATH_TRACER_SHAPE_HPP

#include "../core/ray.hpp"
#include "../bvh/axisAlignedBox.hpp"
#include "../core/random.hpp"
#include "../math/mathUtil.hpp"

namespace pathtracer{

    // Forward declaration
    class Intersection;
    class AreaSample;


    // Contains point sampled from surface, and optionally
    // a triangle index for meshes
    struct SurfaceSample{
        Vector3 point;
        int triangleIndex;
    };

    class Shape{

    public:

        Shape() {} 
        

        // Returns surface area of the shape, in the space it is
        // defined in.
        virtual real getSurfaceArea() const = 0;


        // Returns a bounding box, in the space that the shape is
        // defined in.
        virtual AxisAlignedBox getBoundingBox() const = 0;


        // Returns the centroid
        virtual Vector3 getCentroid() const = 0;


        // Intersects the shape with a ray.
        // We assume the ray is in the correct coordinate system
        // prior to calling the function.
        // As input we send the last closest t (transformed into local
        // coordinates), and we can optimize this function by only returning
        // a valid intersection when it is closer than the oldT
        // (though it will also be checked later just in case).
        // If multiple hits are found, only the closest is returned.
        // Visibility is not checked at this level, but self intersection is.
        virtual Intersection intersect(const Ray&, real oldT) const = 0;


        // Samples a random point on the surface area
        virtual AreaSample sampleSurfaceArea() const = 0;


        // Returns the Area sample for a given point on the surface.
        // The point is expected in local coordinates.
        virtual AreaSample evaluateAreaSample(const SurfaceSample&) const = 0;

    };
}

#endif