

#ifndef PATH_TRACER_INSTANCE_HPP
#define PATH_TRACER_INSTANCE_HPP

#include "../shapes/shape.hpp"
#include "../texture/texture.hpp"
#include "../bsdf/bsdf.hpp"
#include "../emission/emission.hpp"
#include "transform.hpp"

namespace pathtracer{

    // Forward declaration
    class AreaSample;
    class Light;
    class Scene;

    class Instance{

    private:

        const Shape* m_shape;
        const Texture* m_alpha;
        const Texture* m_normal;
        const Bsdf* m_bsdf;
        const Emission* m_emission;
        Transform m_transform;

        // If this current instance is considered an area light,
        // then this pointer points to it.
        Light* m_light;

        // This is true if the current instance is intersectable
        // in the scene.
        bool m_inScene;


        // The scene is a friend class
        friend class Scene;

        
        void setInScene(bool inScene){
            m_inScene = inScene;
        }

    public:


        Instance(const Shape* shape,
            const Texture* alpha, const Texture* normal,
            const Bsdf* bsdf, const Emission* emission, 
            const Transform& transform) : 
            m_shape(shape), m_alpha(alpha), m_normal(normal),
            m_bsdf(bsdf), m_emission(emission),
            m_transform(transform), m_light{nullptr}, m_inScene{false} {}

        
        bool hasAlphaTexture() const{
            return m_alpha != nullptr;
        }


        bool hasNormalTexture() const{
            return m_normal != nullptr;
        }


        bool inScene() const {return m_inScene; }


        // Transforms shape's box to global coordinates and then finds
        // AABB that fits it (not the tightest fit but fast).
        AxisAlignedBox getBoundingBox() const{

            AxisAlignedBox localBounds = m_shape->getBoundingBox();

            Vector3 min = localBounds.minCorner();
            Vector3 max = localBounds.maxCorner();

            AxisAlignedBox result;

            result.extend(m_transform.transform(Vector3(min.x(), min.y(), min.z())));
            result.extend(m_transform.transform(Vector3(max.x(), min.y(), min.z())));
            result.extend(m_transform.transform(Vector3(min.x(), max.y(), min.z())));
            result.extend(m_transform.transform(Vector3(max.x(), max.y(), min.z())));

            result.extend(m_transform.transform(Vector3(min.x(), min.y(), max.z())));
            result.extend(m_transform.transform(Vector3(max.x(), min.y(), max.z())));
            result.extend(m_transform.transform(Vector3(min.x(), max.y(), max.z())));
            result.extend(m_transform.transform(Vector3(max.x(), max.y(), max.z())));

            return result;

        }


        const Texture* const alphaTexture() const{
            return m_alpha;
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
        

        void setLight(Light* light){
            m_light = light;
        }


        Light* light() const{ return m_light; }


        // Samples random point on surface area, and returns the
        // result in world coordinates, using the area measure pdf
        AreaSample sampleArea() const;


        // Returns, in global coordinates, the area sample of sampling
        // a point on the surface of the instance.
        // The input is in global coordinates.
        AreaSample evaluateAreaSample(const SurfaceSample&) const;
        
    };

}

#endif