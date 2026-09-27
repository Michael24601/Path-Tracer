
#ifndef PATH_TRACER_SURFACE_POINT_HPP
#define PATH_TRACER_SURFACE_POINT_HPP

#include "math/vector3.hpp"
#include "math/vector2.hpp"
#include "core/transform.hpp"

namespace pathtracer{

    class Instance;
    class BsdfSample;

    // Class that represents a surface point, that is, a point
    // on the surface of a shape that either by area sampling or ray
    // intersection, was chosen.
    class SurfacePoint{

    protected:

        Vector3 m_position;

        Vector3 m_geometryNormal;

        Vector3 m_shadingNormal;

        Vector3 m_tangent;

        // Texture coordinates
        Vector2 m_uv;

        const Instance* m_instance;

        // The shading frame that consists of the bitangent, tangent
        // and normals, where the normal is the z coordinate.
        Transform m_shadingFrame;

        // Optional parameter, records which triangle in a mesh was
        // intersected.
        int m_triangleIndex;

    public:

        SurfacePoint();

        SurfacePoint(
            const Vector3& position,
            const Vector3& geometryNormal,
            const Vector3& shadingNormal,
            const Vector3& tangent,
            const Vector2& uv,
            const Instance* instance);

        const Vector3& position() const;

        const Vector3& geometryNormal() const;

        const Vector3& shadingNormal() const;

        const Vector3& tangent() const;

        const Vector2& uv() const;

        const Instance* instance() const;

        const Transform& shadingFrame() const;

        int triangleIndex() const;

        void setPosition(const Vector3& pos);

        void setGeometryNormal(const Vector3& normal);

        void setShadingNormal(const Vector3& normal);

        void setTangent(const Vector3& tangent);

        void setUV(const Vector2& uv);

        void setInstance(const Instance* instance);

        void setTriangleIndex(int index);

        void computeShadingFrame();

        // Samples the BSDF of the instance at this point.
        // We assume the direction wo is given in world coordinates.
        // This also returns the sample in world coordinates.
        BsdfSample sampleBsdf(const Vector3& wo) const;

        // Evaluates the BSDF of the instance at this point.
        // We assume the direction wo is given in world coordinates.
        // We are given the wi already, so there is no need to
        // sample anything.
        // This also returns the evaluation in world coordinates.
        BsdfSample evaluateBsdf(
            const Vector3& wo,
            const Vector3& wi) const;

        // Here, we just evaluate the emission at this point.
        // Returns the emission in world coordinates
        Vector3 evaluateEmission(const Vector3& wo) const;
    };

}

#endif