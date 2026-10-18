#ifndef PATH_TRACER_AXIS_ALIGNED_BOX_HPP
#define PATH_TRACER_AXIS_ALIGNED_BOX_HPP

#include "math/vector3.hpp"

namespace pathtracer{

    class Ray;

    class AxisAlignedBox{

    private:

        Vector3 m_minCorner;
        Vector3 m_maxCorner;

    public:

        AxisAlignedBox();

        AxisAlignedBox(const Vector3& minCorner, const Vector3& maxCorner);

        // Increases the AABB size to fit another AABB
        void extend(const AxisAlignedBox& bounds);

        // Increases AABB size to fit a point
        void extend(const Vector3& point);

        // Scales the box, relative to its center
        void scale(real scale){
            Vector3 center = (m_minCorner + m_maxCorner) * 0.5;
            Vector3 halfSize = (m_maxCorner - m_minCorner) * 0.5 * scale;

            m_minCorner = center - halfSize;
            m_maxCorner = center + halfSize;
        }

        real surfaceArea() const;


        const Vector3& minCorner() const {
            return m_minCorner;
        }


        const Vector3& maxCorner() const {
            return m_maxCorner;
        }


        void setMaxCorner(const Vector3& maxCorner){
            m_maxCorner = maxCorner;
        }

        void setMinCorner(const Vector3& minCorner){
            m_minCorner = minCorner;
        }

        real intersect(const Ray& ray) const;

    };

}

#endif