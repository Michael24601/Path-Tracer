

#ifndef PATH_TRACER_SURFACE_POINT_HPP
#define PATH_TRACER_SURFACE_POINT_HPP

#include "../math/vector3.hpp"
#include "../math/vector2.hpp"
#include "../core/instance.hpp"
#include "../bsdf/bsdfSample.hpp"

namespace pathtracer{

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
        

    public:


        SurfacePoint(const Vector3& position, 
            const Vector3& geometryNormal, const Vector3& shadingNormal,
            const Vector3& tangent, const Vector2& uv, 
            const Instance* instance) : 
            m_position(position), 
            m_geometryNormal(geometryNormal), 
            m_shadingNormal(shadingNormal),
            m_tangent(tangent),
            m_uv(uv),
            m_instance{instance} {};


        const Vector3& position() const { return m_position; }


        const Vector3& geometryNormal() const { return m_geometryNormal; }
        
        
        const Vector3& shadingNormal() const { return m_shadingNormal; }


        const Vector3& tangent() const { return m_tangent; }

    
        const Vector2& uv() const { return m_uv; }

        
        const Instance* instance() const { return m_instance; }


        void setPosition(const Vector3& pos) { m_position = pos; }

        
        void setGeometryNormal(const Vector3& normal) { 
            m_geometryNormal = normal; 
        }


        void setShadingNormal(const Vector3& normal) { 
            m_shadingNormal = normal; 
        }


        void setTangent(const Vector3& tangent){
            m_tangent = tangent;
        }


        void setUV(const Vector2& uv) { m_uv = uv; }


        void setInstance(const Instance* instance) { 
            m_instance = instance; 
        }


        void computeShadingFrame(){
            // Bitangent
            Vector3 bit = m_shadingNormal.cross(m_tangent);

            Matrix3 frame = Matrix3(m_tangent, bit, m_shadingNormal);
            m_shadingFrame = Transform(frame, Vector3::ORIGIN);
        }


        // Samples the BSDF of the instance at this point.
        // We assume the direction wo is given in world coordinates.
        BsdfSample sampleBsdf(const Vector3& wo) const{

            if(!m_instance || !m_instance->bsdf()){
                return BsdfSample::INVALID;
            }

            // We transform the direction wo to local coordinates
            Vector3 localWo = m_shadingFrame.inverseTransformDirection(wo);
            BsdfSample sample = m_instance->bsdf()->sample(localWo, m_uv);

            // We then transform the result back to world coordinates
            Vector3 wi = m_shadingFrame.transformDirection(sample.wi());
            sample.setWi(wi);

            // The pdf and bsdf remain the same. The cosine term
            // also remains the same since the shading frame is
            // orthonormal.
            return sample;
        }


        // Evaluates the BSDF of the instance at this point.
        // We assume the direction wo is given in world coordinates.
        // We are given the wi already, so there is no need to
        // sample anything.
        BsdfEvaluation evaluateBsdf(const Vector3& wo, 
            const Vector3& wi) const{

            if(!m_instance || !m_instance->bsdf()){
                return BsdfEvaluation::INVALID;
            }

            // We transform the direction wo and wi to local coordinates
            Vector3 localWo = m_shadingFrame.inverseTransformDirection(wo);
            Vector3 localWi = m_shadingFrame.inverseTransformDirection(wi);

            // No need to transform anything else
            BsdfEvaluation eval = m_instance->bsdf()->evaluate(localWo, 
                localWi, m_uv);

            // The pdf and bsdf remain the same. The cosine term
            // also remains the same since the shading frame is
            // orthonormal.
            return eval;
        }


        // Here, we just evaluate the emission at this point.
        Vector3 evaluateEmission(const Vector3& wo) const{

            if(!m_instance || !m_instance->emission()){
                return Vector3::ORIGIN;
            }

            Vector3 localWo = m_shadingFrame.inverseTransformDirection(wo);
            return m_instance->emission()->evaluate(wo, m_uv);
        }

    };

}

#endif