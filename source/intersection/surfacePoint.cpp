
#include "intersection/surfacePoint.hpp"
#include "bsdf/bsdfSample.hpp"
#include "bsdf/bsdf.hpp"
#include "core/instance.hpp"
#include "emission/emission.hpp"
#include "math/matrix3.hpp"

namespace pathtracer{

    SurfacePoint::SurfacePoint() :
        m_position(Vector3::ORIGIN),
        m_geometryNormal(Vector3::ORIGIN),
        m_shadingNormal(Vector3::ORIGIN),
        m_baseNormal{Vector3::ORIGIN},
        m_uv(Vector2::ORIGIN),
        m_instance(nullptr),
        m_triangleIndex{-1} {}


    SurfacePoint::SurfacePoint(const Vector3& position,
        const Vector3& geometryNormal, const Vector3& shadingNormal,
        const Vector2& uv, const SurfaceDifferentials& differentials, 
        const Instance* instance) :
        m_position(position),
        m_geometryNormal(geometryNormal),
        m_shadingNormal(shadingNormal),
        m_baseNormal(shadingNormal),
        m_differentials(differentials),
        m_uv(uv),
        m_instance{instance},
        m_triangleIndex{-1} {}


    const Vector3& SurfacePoint::position() const {
        return m_position;
    }


    const Vector3& SurfacePoint::geometryNormal() const {
        return m_geometryNormal;
    }


    const Vector3& SurfacePoint::shadingNormal() const {
        return m_shadingNormal;
    }


    const Vector3& SurfacePoint::baseNormal() const {
        return m_baseNormal;
    }


    Vector3 SurfacePoint::s() const{
        return m_shadingFrame.transform().column(0);
    }

    
    Vector3 SurfacePoint::t() const{
        return m_shadingFrame.transform().column(1);
    }


    const Vector2& SurfacePoint::uv() const {
        return m_uv;
    }


    const Instance* SurfacePoint::instance() const {
        return m_instance;
    }


    const Transform& SurfacePoint::shadingFrame() const {
        return m_shadingFrame;
    }


    const Transform& SurfacePoint::baseFrame() const {
        return m_baseFrame;
    }


    const SurfaceDifferentials& SurfacePoint::differentials() const{
        return m_differentials;
    }


    int SurfacePoint::triangleIndex() const {
        return m_triangleIndex;
    }


    void SurfacePoint::setPosition(const Vector3& pos) {
        m_position = pos;
    }


    void SurfacePoint::setGeometryNormal(const Vector3& normal) {
        m_geometryNormal = normal;
    }


    void SurfacePoint::setShadingNormal(const Vector3& normal) {
        m_shadingNormal = normal;
    }


    void SurfacePoint::setBaseNormal(const Vector3& normal) {
        m_baseNormal = normal;
    }


    void SurfacePoint::setUV(const Vector2& uv) {
        m_uv = uv;
    }


    void SurfacePoint::setInstance(const Instance* instance) {
        m_instance = instance;
    }


    void SurfacePoint::setTriangleIndex(int index) {
        m_triangleIndex = index;
    }


    void SurfacePoint::computeBaseFrame() {

        // Computes tangent from dpdu
        Vector3 s = (m_differentials.dpdu() - m_baseNormal 
            * m_baseNormal.dot(m_differentials.dpdu())).normalized();

        // Bitangent
        Vector3 bit = m_baseNormal.cross(s);

        Matrix3 frame = Matrix3(s, bit, m_baseNormal);

        m_baseFrame = Transform(frame, m_position);
    }


    void SurfacePoint::computeShadingFrame() {

        // Computes tangent from dpdu
        Vector3 s = (m_differentials.dpdu() - m_shadingNormal 
            * m_shadingNormal.dot(m_differentials.dpdu())).normalized();

        // Bitangent
        Vector3 bit = m_shadingNormal.cross(s);

        Matrix3 frame = Matrix3(s, bit, m_shadingNormal);

        m_shadingFrame = Transform(frame, m_position);
    }


    // Samples the BSDF of the instance at this point.
    // We assume the direction wo is given in world coordinates.
    // This also returns the sample in world coordinates.
    BsdfSample SurfacePoint::sampleBsdf(const Vector3& wo, 
        bool adjoint) const {

        if(!m_instance || !m_instance->bsdf()){
            return BsdfSample::INVALID;
        }

        // We transform the direction wo to local coordinates
        Vector3 localWo = m_shadingFrame.inverseTransformDirection(wo);

        // The bsdf functions sample and evaluate in shading
        // frame coordinates, so we need to transform the inputs,
        // and then make any necessary modifications to the output
        // as well.
        BsdfSample sample = m_instance->bsdf()->sample(localWo, m_uv, adjoint);

        // We then transform the result back to world coordinates
        Vector3 wi = m_shadingFrame.transformDirection(sample.wi());

        sample.setWi(wi);

        // The pdf and bsdf remain the same in world coordinates.
        // The cosine term also remains the same since the shading
        // frame is orthogonal.
        return sample;
    }


    // Evaluates the BSDF of the instance at this point.
    // We assume the direction wo is given in world coordinates.
    // We are given the wi already, so there is no need to
    // sample anything.
    // This also returns the evaluation in world coordinates.
    BsdfSample SurfacePoint::evaluateBsdf(const Vector3& wo,
        const Vector3& wi, bool adjoint) const {

        if(!m_instance || !m_instance->bsdf()){
            return BsdfSample::INVALID;
        }

        // We transform the directions wo and wi to local coordinates
        Vector3 localWo = m_shadingFrame.inverseTransformDirection(wo);

        Vector3 localWi = m_shadingFrame.inverseTransformDirection(wi);

        // No need to transform anything else
        BsdfSample sample = m_instance->bsdf()->evaluate(localWo, localWi, 
            m_uv, adjoint);

        sample.setWi(wi);

        // The pdf and bsdf remain the same in world coordinates.
        // The cosine term also remains the same since the shading
        // frame is orthogonal.
        return sample;
    }


    // Here, we just evaluate the emission at this point.
    // Returns the emission in world coordinates
    Vector3 SurfacePoint::evaluateEmission(const Vector3& wo) const {

        if(!m_instance || !m_instance->emission()){
            return Vector3::ORIGIN;
        }

        Vector3 localWo = m_shadingFrame.inverseTransformDirection(wo);

        // The emission result is the same in world and shading
        // frame coordinates.
        return m_instance->emission()->evaluate(localWo, m_uv);
    }


}