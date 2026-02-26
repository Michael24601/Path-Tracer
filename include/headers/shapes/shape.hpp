
#ifndef PATH_TRACER_SHAPE_HPP
#define PATH_TRACER_SHAPE_HPP

#include "config.hpp"
#include "../core/ray.hpp"
#include "../intersection/intersection.hpp"
#include "../core/areaSample.hpp"
#include "../bvh/axisAlignedBox.hpp"
#include "../core/random.hpp"
#include "../math/mathUtil.hpp"

namespace pathtracer{

    enum class Space { 
        LOCAL,
        GLOBAL
    };


    class Shape{

    private:
        
        Space m_space;

    public:

        Shape(Space space) : m_space{space}{} 


        virtual Space space() const { return m_space; }


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
        virtual std::vector<Intersection> intersect(const Ray&) const = 0;


        // Samples a random point on the surface area
        virtual AreaSample sampleSurfaceArea() const = 0;

    };
}

#endif