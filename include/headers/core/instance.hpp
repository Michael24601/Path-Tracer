

#ifndef PATH_TRACER_INSTANCE_HPP
#define PATH_TRACER_INSTANCE_HPP

#include "../shapes/shape.hpp"
#include "../texture/texture.hpp"
#include "../bsdf/bsdf.hpp"
#include "../emission/emission.hpp"
#include "transform.hpp"

namespace pathtracer{

    class Instance{

    private:

        const Shape* m_shape;
        const Texture* m_alpha;
        const Texture* m_normal;
        const Texture* m_albedo;
        const Bsdf* m_bsdf;
        const Emission* m_emission;
        Transform m_transform;

    public:


        Instance(const Shape* shape,
            const Texture* alpha, const Texture* normal,
            const Texture* albedo, const Bsdf* bsdf,
            const Emission* emission, const Transform& transform) : 
            m_shape(shape), m_alpha(alpha), m_normal(normal),
            m_albedo(albedo), m_bsdf(bsdf), m_emission(emission),
            m_transform(transform) {}

        
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


        const Emission* const emission() const{
            return m_emission;
        }


        const Bsdf* const bsdf() const {
            return m_bsdf;
        }
        

    };

}

#endif