

#ifndef PATH_TRACER_INTERSECTION_HPP
#define PATH_TRACER_INTERSECTION_HPP

#include "vector3.hpp"
#include "vector2.hpp"
#include "shape.hpp"
#include "instance.hpp"

namespace pathtracer{

    class Intersection{

    private:

        // Distance along the ray that the intersection took place.
        real m_t;

        Vector3 m_position;

        Vector3 m_geometryNormal;

        Vector3 m_shadingNormal;

        // Texture coordinates
        Vector2 m_uv;

        const Instance* m_instance;
        

    public:

        static Intersection NO_HIT;


        Intersection(real t, const Vector3& position, 
            const Vector3& geometryNormal, const Vector3& shadingNormal, 
            const Vector2& uv, const Instance* instance) : 
            m_t{t}, m_position(position), 
            m_geometryNormal(geometryNormal), 
            m_shadingNormal(shadingNormal),
            m_uv(uv),
            m_instance{instance} {};


        const real t() const { return m_t; }


        const Vector3& position() const { return m_position; }


        const Vector3& geometryNormal() const { return m_geometryNormal; }
        
        
        const Vector3& shadingNormal() const { return m_shadingNormal; }

    
        const Vector2& uv() const { return m_uv; }

        
        const Instance* instance() const { return m_instance; }

    };

}

#endif