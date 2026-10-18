#ifndef PATH_TRACER_AXIS_ALIGNED_RECTANGLE_HPP
#define PATH_TRACER_AXIS_ALIGNED_RECTANGLE_HPP

#include "math/vector2.hpp"

namespace pathtracer{

    class Ray;

    class AxisAlignedRectangle{

    private:

        Vector2 m_minCorner;
        Vector2 m_maxCorner;

    public:

        AxisAlignedRectangle();

        AxisAlignedRectangle(const Vector2& minCorner, const Vector2& maxCorner);

        // Increases the AABB size to fit another AABB
        void extend(const AxisAlignedRectangle& bounds);

        // Increases AABB size to fit a point
        void extend(const Vector2& point);

        // Scales the box, relative to its center
        void scale(real scale){
            Vector2 center = (m_minCorner + m_maxCorner) * 0.5;
            Vector2 halfSize = (m_maxCorner - m_minCorner) * 0.5 * scale;

            m_minCorner = center - halfSize;
            m_maxCorner = center + halfSize;
        }


        const Vector2& minCorner() const {
            return m_minCorner;
        }


        const Vector2& maxCorner() const {
            return m_maxCorner;
        }


        void setMaxCorner(const Vector2& maxCorner){
            m_maxCorner = maxCorner;
        }

        void setMinCorner(const Vector2& minCorner){
            m_minCorner = minCorner;
        }

    };

}

#endif