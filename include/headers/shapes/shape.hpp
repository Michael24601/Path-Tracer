
#ifndef PATH_TRACER_SHAPE_HPP
#define PATH_TRACER_SHAPE_HPP

#include "../core/ray.hpp"
#include "../bvh/axisAlignedBox.hpp"
#include "../core/random.hpp"
#include "../math/mathUtil.hpp"

namespace pathtracer{

    // Forward declaration
    class IntersectionList;
    class AreaSample;

    class Shape{

    public:

        Shape() {} 
        

        // Returns surface area of the shape, in the space it is
        // defined in.
        virtual real getSurfaceArea() const = 0;


        // Returns a bounding box, in the space that the shape is
        // defined in.
        virtual AxisAlignedBox getBoundingBox() const = 0;


        // Intersects the shape with a ray.
        // We assume the ray is in the correct coordinate system
        // prior to calling the function.
        // Note that the intersection only checks if the ray
        // intersects the object. It does not check for visibility,
        // whether the object is behind the ray, or for self inetrsection.
        virtual void intersect(const Ray&, IntersectionList&) const = 0;


        // Samples a random point on the surface area
        virtual AreaSample sampleSurfaceArea() const = 0;

    };
}

#endif