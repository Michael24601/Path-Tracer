
#include "bvh/axisAlignedRectangle.hpp"
#include "math/constants.hpp"
#include <cassert>

namespace pathtracer{

    AxisAlignedRectangle::AxisAlignedRectangle() : m_minCorner(REAL_INFINITY), 
        m_maxCorner(-REAL_INFINITY) {}


    AxisAlignedRectangle::AxisAlignedRectangle(
        const Vector2& minCorner, const Vector2& maxCorner) : 
        m_minCorner(minCorner), m_maxCorner(maxCorner) {

        assert((minCorner <= maxCorner) 
            && "The corner coordinates are not ordered right");
    }


    // Increases the AABB size to fit another AABB
    void AxisAlignedRectangle::extend(const AxisAlignedRectangle& bounds) {
        m_minCorner = m_minCorner.min(bounds.m_minCorner);
        m_maxCorner = m_maxCorner.max(bounds.m_maxCorner);
    }


    // Increases AABB size to fit a point
    void AxisAlignedRectangle::extend(const Vector2& point) {
        m_minCorner = m_minCorner.min(point);
        m_maxCorner = m_maxCorner.max(point);
    }

}