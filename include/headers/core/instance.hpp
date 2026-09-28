
#ifndef PATH_TRACER_INSTANCE_HPP
#define PATH_TRACER_INSTANCE_HPP

#include "core/transform.hpp"

namespace pathtracer{

    class Shape;
    class Texture;
    class Bsdf;
    class Emission;
    class Light;
    class Scene;
    class AreaSample;
    class SurfaceDifferentials;
    class SurfaceSample;
    class SurfacePoint;
    class AxisAlignedBox;
    class Vector2;

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

        // This is true if the current instance is marked as a caustic
        // receiver.
        bool m_isCausticReceiver;

        // The scene is a friend class
        friend class Scene;

        
        void setInScene(bool inScene);

    public:


        Instance(const Shape* shape,
            const Texture* alpha, const Texture* normal,
            const Bsdf* bsdf, const Emission* emission, 
            const Transform& transform);

        
        bool hasAlphaTexture() const;


        bool hasNormalTexture() const;


        bool inScene() const;


        // Transforms shape's box to global coordinates and then finds
        // AABB that fits it (not the tightest fit but fast).
        AxisAlignedBox getBoundingBox() const;


        const Texture* const alphaTexture() const;


        const Texture* const normalTexture() const;


        const Transform& transform() const;


        const Shape* const shape() const;


        const Emission* const emission() const;


        const Bsdf* const bsdf() const;
        

        void setLight(Light* light);


        void setCausticReceiver(bool isCausticReceiver);


        bool isCausticReceiver() const;


        Light* light() const;


        // Samples random point on surface area, and returns the
        // result in world coordinates, using the area measure pdf
        AreaSample sampleArea() const;


        // Returns, in global coordinates, the area sample of sampling
        // a point on the surface of the instance.
        // The input is in global coordinates.
        AreaSample evaluateAreaSample(const SurfaceSample&) const;


        // Computes the differential info on the surface of the shape
        // (gets surface point in global coordinates)
        SurfaceDifferentials computeDifferentials(const SurfacePoint&) const;


        // Returns position given uv coordinates
        Vector3 getPosition(const Vector2& uv, int triangleIndex) const;

    };

}

#endif