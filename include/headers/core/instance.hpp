

#ifndef PATH_TRACER_INSTANCE_HPP
#define PATH_TRACER_INSTANCE_HPP

#include "shape.hpp"
#include "transform.hpp"
#include "../texture/texture.hpp"

namespace pathtracer{

    class Instance{

    private:

        const Shape* m_shape;
        const Texture* m_alpha;
        const Texture* m_normal;
        const Texture* m_albedo;
        Transform m_transform;

    public:

        
        bool hasAlphaTexture() const{
            return m_alpha != nullptr;
        }


        bool hasAlbedoTexture() const{
            return m_albedo != nullptr;
        }


        bool hasNormalTexture() const{
            return m_normal != nullptr;
        }


        const Texture* const alphaTexture() const{
            return m_alpha;
        }

        
        const Texture* const albedoTexture() const{
            return m_albedo;
        }


        const Texture* const normalTexture() const{
            return m_normal;
        }


        const Transform& transform() const{
            return m_transform;
        }


        const Shape* const shape() const{
            return m_shape;
        }
        

    };

}

#endif