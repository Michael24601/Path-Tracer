
#ifndef PATH_TRACER_SURFACE_POINT_HPP
#define PATH_TRACER_SURFACE_POINT_HPP

#include "math/vector3.hpp"
#include "math/vector2.hpp"
#include "core/transform.hpp"
#include <string>

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

        // Because tangent can be modified by normal mapping, this tangent
        // stays immutable.
        Vector3 m_geometryTangent;

        Vector3 m_shadingNormal;

        Vector3 m_tangent;

        // The shading normal, never normal mapped
        Vector3 m_baseNormal;

        // Texture coordinates
        Vector2 m_uv;

        const Instance* m_instance;

        // The shading frame that consists of the bitangent, tangent
        // and normals, where the normal is the z coordinate.
        Transform m_shadingFrame;

        // The same but using geometry normal. Should never be modified.
        Transform m_geometryFrame;

        // Optional parameter, records which triangle in a mesh was
        // intersected.
        int m_triangleIndex;

    public:

        SurfacePoint();

        SurfacePoint(const Vector3& position,
            const Vector3& geometryNormal,
            const Vector3& shadingNormal,
            const Vector3& geometryTangent,
            const Vector3& tangent,
            const Vector2& uv,
            const Instance* instance);

        const Vector3& position() const;

        const Vector3& geometryNormal() const;

        const Vector3& baseNormal() const;

        const Vector3& shadingNormal() const;

        const Vector3& geometryTangent() const;

        const Vector3& tangent() const;

        const Vector2& uv() const;

        const Instance* instance() const;

        const Transform& shadingFrame() const;

        const Transform& geometryFrame() const;

        int triangleIndex() const;

        void setPosition(const Vector3& pos);

        void setGeometryNormal(const Vector3& normal);

        void setShadingNormal(const Vector3& normal);
    
        void setBaseNormal(const Vector3& normal);

        void setTangent(const Vector3& tangent);

        void setUV(const Vector2& uv);

        void setInstance(const Instance* instance);

        void setTriangleIndex(int index);

        void computeGeometryFrame();

        void computeShadingFrame();

        // Samples the BSDF of the instance at this point.
        // We assume the direction wo is given in world coordinates.
        // This also returns the sample in world coordinates.
        BsdfSample sampleBsdf(const Vector3& wo, bool adjoint = false) const;

        // Evaluates the BSDF of the instance at this point.
        // We assume the direction wo is given in world coordinates.
        // We are given the wi already, so there is no need to
        // sample anything.
        // This also returns the evaluation in world coordinates.
        BsdfSample evaluateBsdf(const Vector3& wo, const Vector3& wi, 
            bool adjoint = false) const;

        // Here, we just evaluate the emission at this point.
        // Returns the emission in world coordinates
        Vector3 evaluateEmission(const Vector3& wo) const;


        std::string toString() const;
    };

}

#endif