
#include "core/transform.hpp"
#include "core/ray.hpp"
#include "intersection/surfaceDifferentials.hpp"
#include "intersection/surfacePoint.hpp"

namespace pathtracer{


    Transform Transform::IDENTITY = Transform( 
        Matrix3(
            Vector3(1.0,0.0,0.0),
            Vector3(0.0,1.0,0.0),
            Vector3(0.0,0.0,1.0)
        ), Vector3(0,0,0)
    );


    Transform::Transform() {
        m_transform = Matrix3::IDENTITY;
        m_translation = Vector3::ORIGIN;
    }


    Transform::Transform(const Vector3& angles, const Vector3& scale,
        const Vector3& translation) : m_translation(translation) {

        for(int i = 0; i <= 2; i++){
            m_transform.addRotation(angles[i], i);
        }

        for(int i = 0; i <= 2; i++){
            m_transform.scale(scale[i], i);
        }
    }


    Transform::Transform(const Matrix3& transform,
        const Vector3& translation) :
        m_translation(translation), m_transform(transform) {}


    const Matrix3& Transform::getMatrix() const {
        return m_transform;
    }


    Transform Transform::inverse() const {
        Matrix3 invM = m_transform.inverse();
        Vector3 invT = -(invM * m_translation);
        return Transform(invM, invT);
    }


    Transform Transform::inverseTranspose() const {
        Matrix3 invM = m_transform.inverse().transposed();
        Vector3 t = Vector3::ORIGIN;
        return Transform(invM, t);
    }


    Vector3 Transform::transform(const Vector3& point) const {
        return (m_transform * point) + m_translation;
    }


    const Matrix3& Transform::transform() const {
        return m_transform;
    }


    const Vector3& Transform::translation() const {
        return m_translation;
    }


    Vector3 Transform::inverseTransform(const Vector3& point) const {
        Matrix3 m = m_transform.inverse();
        return (m * point) - (m * m_translation);
    }


    Vector3 Transform::transformDirection(const Vector3& direction) const {
        return (m_transform * direction).normalized();
    }


    Vector3 Transform::transformDirectionKeepScale(
        const Vector3& direction) const {
        return (m_transform * direction);
    }


    Vector3 Transform::inverseTransformDirection(
        const Vector3& direction) const {
        Matrix3 m = m_transform.inverse();
        return (m * direction).normalized();
    }


    Vector3 Transform::transformNormal(const Vector3& normal) const {
        return (m_transform.inverse().transposed() * normal).normalized();
    }


    Vector3 Transform::transformNormalKeepScale(
        const Vector3& normal) const {
        return (m_transform.inverse().transposed() * normal);
    }


    Vector3 Transform::inverseTransformNormal(
        const Vector3& normal) const {
        return (m_transform.transposed() * normal).normalized();
    }


    Ray Transform::transform(const Ray& ray) const {
        Ray r(transform(ray.origin()),
            transformDirection(ray.direction()));
        return r;
    }


    Ray Transform::transformKeepScale(const Ray& ray) const {
        Ray r(transform(ray.origin()),
            transformDirectionKeepScale(ray.direction()));
        return r;
    }


    Ray Transform::inverseTransform(const Ray& ray) const {
        Ray r(inverseTransform(ray.origin()),
            inverseTransformDirection(ray.direction()));
        return r;
    }


    real Transform::determinant() const {
        return m_transform.determinant();
    }


    SurfacePoint Transform::transformSurfacePoint(const SurfacePoint& it) 
        const{

        SurfacePoint res(
            transform(it.position()),
            transformNormal(it.geometryNormal()),
            transformNormal(it.shadingNormal()),
            transformDirection(it.tangent()),
            it.uv(),
            it.instance());

        // Not in constructor must be set explicitely
        res.setTriangleIndex(it.triangleIndex());
            
        return res;
    }


    SurfaceDifferentials Transform::transformDifferentials(
        const SurfaceDifferentials& d, const Vector3& localNormal,
        const Vector3& worldNormal) const {

        Vector3 q = m_transform.inverse().transposed() * localNormal;
        real length = q.length();

        Vector3 dndu = m_transform.inverse().transposed() * d.dndu();
        Vector3 dndv = m_transform.inverse().transposed() * d.dndv();

        // We need to take into account normal normalization
        dndu = (dndu - worldNormal * worldNormal.dot(dndu)) / length;
        dndv = (dndv - worldNormal * worldNormal.dot(dndv)) / length;

        Vector3 dpdu = transformDirectionKeepScale(d.dpdu());
        Vector3 dpdv = transformDirectionKeepScale(d.dpdv());

        Vector3 d2pdu2 = transformDirectionKeepScale(d.d2pdu2());
        Vector3 d2pdudv = transformDirectionKeepScale(d.d2pdudv());
        Vector3 d2pdv2 = transformDirectionKeepScale(d.d2pdv2());

        // Instead of transforming s, it is easier to just recompute it
        Vector3 s = (dpdu - worldNormal * worldNormal.dot(dpdu)).normalized();

        return SurfaceDifferentials(
            dpdu, dpdv, dndu, dndv,
            d2pdu2, d2pdudv, d2pdv2, s
        );
        
    }


    std::string Transform::toString() const {
        return "Transform(\n"
            + m_transform.toString() + "\n"
            + m_translation.toString() + "\n)";
    }

}