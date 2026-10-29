
#ifndef PATH_TRACER_SURFACE_POINT_HPP
#define PATH_TRACER_SURFACE_POINT_HPP

#include "math/vector3.hpp"
#include "math/vector2.hpp"
#include "core/transform.hpp"
#include "surfaceDifferentials.hpp"
#include <string>

namespace pathtracer{

    class Instance;
    class BsdfSample;


    // A Transform that knows it is orthonormal, and writes to world
    // and to local transforms that are faster.
    struct Frame {
        Vector3 s, t, n, origin;

        // To be used with directions only
        Vector3 toLocal(const Vector3& v) const {
            return Vector3(s.dot(v), t.dot(v), n.dot(v));
        }

        // To be used with direction only
        Vector3 toWorld(const Vector3& v) const {
            return s * v.x() + t * v.y() + n * v.z();
        }
    };


    // Class that represents a surface point, that is, a point
    // on the surface of a shape that either by area sampling or ray
    // intersection, was chosen.
    class SurfacePoint{

    protected:

        Vector3 m_position;

        Vector3 m_geometryNormal;

        Vector3 m_shadingNormal;

        // The shading normal, never normal mapped
        Vector3 m_baseNormal;

        // Texture coordinates
        Vector2 m_uv;

        // The differentials dpdu, dpdv, dndu... at this point.
        // Computed at the shape level and transformed to world coordinates.
        // With respect to base normal. 
        // These values DO NOT change when normal mapping, as they are
        // based on the base normal.
        SurfaceDifferentials m_differentials;

        const Instance* m_instance;

        // The shading frame that consists of the bitangent, tangent
        // and normals, where the normal is the z coordinate.
        Frame m_shadingFrame;

        // The same but using the base shading normal without normal
        // mapping.
        Frame m_baseFrame;

        // Optional parameter, records which triangle in a mesh was
        // intersected.
        int m_triangleIndex;

    public:

        SurfacePoint();

        SurfacePoint(const Vector3& position, const Vector3& geometryNormal, 
            const Vector3& shadingNormal, const Vector2& uv,
            const SurfaceDifferentials& differentials, const Instance* instance);

        const Vector3& position() const;

        const Vector3& geometryNormal() const;

        const Vector3& baseNormal() const;

        const Vector3& shadingNormal() const;

        // This is the tangent, returned from the shading frame
        Vector3 s() const;

        // This is the bitangent, returned from the shading frame
        Vector3 t() const;

        const Vector2& uv() const;

        const Instance* instance() const;

        const Frame& shadingFrame() const;

        const Frame& baseFrame() const;

        const SurfaceDifferentials& differentials() const;

        int triangleIndex() const;

        void setPosition(const Vector3& pos);

        void setGeometryNormal(const Vector3& normal);

        void setShadingNormal(const Vector3& normal);
    
        void setBaseNormal(const Vector3& normal);

        void setUV(const Vector2& uv);

        void setInstance(const Instance* instance);

        void setTriangleIndex(int index);

        void computeBaseFrame();

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


    };

}

#endif