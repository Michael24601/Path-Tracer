
#ifndef PATH_TRACER_AXIS_ALIGNED_BOX_HPP
#define PATH_TRACER_AXIS_ALIGNED_BOX_HPP

#include "vector3.hpp"

namespace pathtracer{

    class AxisAlignedBox{

    private:

        Vector3 m_minCorner;
        Vector3 m_maxCorner;

    public:

        AxisAlignedBox(const Vector3& minCorner, const Vector3& maxCorner) : 
            m_minCorner(minCorner), m_maxCorner(maxCorner){

            assert((minCorner <= maxCorner) 
                && "The corner coordinates are not ordered right");
        }

        
        const Vector3& minCorner() const { return m_minCorner; } 


        const Vector3& maxCorner() const { return m_maxCorner; } 

    };

}

#endif