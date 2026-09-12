
#ifndef PATH_TRACER_AXIS_ALIGNED_BOX_HPP
#define PATH_TRACER_AXIS_ALIGNED_BOX_HPP

#include "../core/ray.hpp"

namespace pathtracer{

    class AxisAlignedBox{

    private:

        Vector3 m_minCorner;
        Vector3 m_maxCorner;

    public:

        AxisAlignedBox() : m_minCorner(REAL_INFINITY), m_maxCorner(-REAL_INFINITY){}


        AxisAlignedBox(const Vector3& minCorner, const Vector3& maxCorner) : 
            m_minCorner(minCorner), m_maxCorner(maxCorner){

            assert((minCorner <= maxCorner) 
                && "The corner coordinates are not ordered right");
        }


        // Increases the AABB size to fit another AABB
        void extend(const AxisAlignedBox& bounds) {
            m_minCorner = m_minCorner.elementWiseMinimum(bounds.m_minCorner);
            m_maxCorner = m_maxCorner.elementWiseMaximum(bounds.m_maxCorner);
        }


        real surfaceArea() const {
            Vector3 size = m_maxCorner - m_minCorner;
            // It's 2 * the area of each plane
            return 2 * (size.x() * size.y()
                    + size.x() * size.z()
                    + size.y() * size.z());
        }

        
        const Vector3& minCorner() const { return m_minCorner; } 


        const Vector3& maxCorner() const { return m_maxCorner; } 

        
        bool intersect(const Ray& ray, real& t) const{

            // First we intersect every single plane in the AABB,
            // and return the distances.
            Vector3 t1 = (m_minCorner - ray.origin()) / ray.direction(); 
            Vector3 t2 = (m_maxCorner - ray.origin()) / ray.direction(); 

            // We then check which is actually closer (element-wise)
            Vector3 tMax = t1.elementWiseMaximum(t2);
            Vector3 tMin = t1.elementWiseMinimum(t2);

            // We then find which axis was actually entered first
            real tNear = tMin.maxElement();
            // And which was left first
            real tFar = tMax.minElement();

            // This condition ensures we intersected the planes inside the AABB
            if (tNear > tFar){
                return false;
            }

            // This ensures we don't intersect the box from behind the ray
            if (tFar < 0){
                return false;
            }

            t = tNear;
            return true;
        }

    };

}

#endif