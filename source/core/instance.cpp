#include "core/instance.hpp"
#include "shapes/shape.hpp"
#include "texture/texture.hpp"
#include "bsdf/bsdf.hpp"
#include "emission/emission.hpp"
#include "bvh/axisAlignedBox.hpp"
#include "math/vector2.hpp"
#include "math/vector3.hpp"
#include "intersection/areaSample.hpp"
#include "intersection/surfaceDifferentials.hpp"


namespace pathtracer{


    void Instance::setInScene(bool inScene) {
        m_inScene = inScene;
    }


    Instance::Instance(const Shape* shape,
        const Texture* alpha, const Texture* normal,
        const Bsdf* bsdf, const Emission* emission, 
        const Transform& transform) : 
        m_shape(shape), 
        m_alpha(alpha), 
        m_normal(normal),
        m_bsdf(bsdf), 
        m_emission(emission),
        m_transform(transform), 
        m_light{nullptr}, 
        m_inScene{false},
        m_isCausticReceiver{false}{}


    bool Instance::hasAlphaTexture() const {
        return m_alpha != nullptr;
    }


    bool Instance::hasNormalTexture() const {
        return m_normal != nullptr;
    }


    bool Instance::inScene() const {
        return m_inScene;
    }


    // Transforms shape's box to global coordinates and then finds
    // AABB that fits it (not the tightest fit but fast).
    AxisAlignedBox Instance::getBoundingBox() const {

        AxisAlignedBox localBounds = m_shape->getBoundingBox();

        Vector3 min = localBounds.minCorner();
        Vector3 max = localBounds.maxCorner();

        AxisAlignedBox result;

        result.extend(m_transform.transform(
            Vector3(min.x(), min.y(), min.z())));

        result.extend(m_transform.transform(
            Vector3(max.x(), min.y(), min.z())));

        result.extend(m_transform.transform(
            Vector3(min.x(), max.y(), min.z())));

        result.extend(m_transform.transform(
            Vector3(max.x(), max.y(), min.z())));

        result.extend(m_transform.transform(
            Vector3(min.x(), min.y(), max.z())));

        result.extend(m_transform.transform(
            Vector3(max.x(), min.y(), max.z())));

        result.extend(m_transform.transform(
            Vector3(min.x(), max.y(), max.z())));

        result.extend(m_transform.transform(
            Vector3(max.x(), max.y(), max.z())));

        return result;
    }


    const Texture* const Instance::alphaTexture() const {
        return m_alpha;
    }


    const Texture* const Instance::normalTexture() const {
        return m_normal;
    }


    const Transform& Instance::transform() const {
        return m_transform;
    }


    const Shape* const Instance::shape() const {
        return m_shape;
    }


    const Emission* const Instance::emission() const {
        return m_emission;
    }


    const Bsdf* const Instance::bsdf() const {
        return m_bsdf;
    }


    void Instance::setCausticReceiver(bool isCausticReceiver){
        m_isCausticReceiver = isCausticReceiver;
    }


    bool Instance::isCausticReceiver() const{
        return m_isCausticReceiver;
    }



    void Instance::setLight(Light* light) {
        m_light = light;
    }


    Light* Instance::light() const {
        return m_light;
    }


    Vector3 Instance::getPosition(const Vector2& uv, int triangleIndex) const {

        Vector3 position = m_shape->getPosition(uv, triangleIndex);
        return m_transform.transform(position);
    }


    // Samples random point on surface area, and returns the
    // result in world coordinates, using the area measure pdf
    AreaSample Instance::sampleArea() const{
        
        // First we sample the shape in local coordinates
        // Both the position and the pdf are in the shape's
        // local coordinates, so we can transform them.
        AreaSample sample = m_shape->sampleSurfaceArea();

        // The transform matrix is used directly since we don't
        // want it normalized yet.
        Transform m = m_transform.inverseTranspose();
        Vector3 normalWorld = m.transform(sample.shadingNormal());

        // Transforming a local pdf to a world pdf
        real worldPdf = sample.pdf() / (normalWorld.length() 
            * m_transform.determinant());

        // We then transform the whole sample
        SurfacePoint newSample = m_transform.transformSurfacePoint(sample);

        // The total pdf is just the pdf of choosing the point
        // multilplied by the pdf of choosing the shape.
        AreaSample areaSample(newSample, worldPdf);

        // We can also set the instance at this point, in the area
        // sample, which is nullptr up tp this point.
        areaSample.setInstance(this);
        areaSample.computeShadingFrame();

        return areaSample;
    }


    AreaSample Instance::evaluateAreaSample(const SurfaceSample& surPoint) const{
        // First we transform the point to local
        // coordinates.
        Vector3 localPoint = m_transform.inverseTransform(surPoint.point);
        SurfaceSample localSample{localPoint, surPoint.triangleIndex};

        // Then we evaluate the area sample.
        // Both the position and the pdf are in the shape's
        // local coordinates, so we can transform them.
        AreaSample sample = m_shape->evaluateAreaSample(localSample);

        // The transform matrix is used directly since we don't
        // want it normalized yet.
        Transform m = m_transform.inverseTranspose();
        Vector3 normalWorld = m.transform(sample.shadingNormal());

        // Transforming a local pdf to a world pdf
        real worldPdf = sample.pdf() / (normalWorld.length() 
            * std::abs(m_transform.determinant()));

        // We then transform the whole sample
        SurfacePoint newSample = m_transform.transformSurfacePoint(sample);
        newSample.setInstance(this);
        newSample.computeShadingFrame();

        // The total pdf is just the pdf of choosing the point
        // multilplied by the pdf of choosing the shape.
        return AreaSample(newSample, worldPdf);   
    }


    SurfaceDifferentials Instance::computeDifferentials(
        const SurfacePoint& sp) const{

        // First we will transform the position and normal to local
        // coordinates.
        Vector3 localPoint = m_transform.inverseTransform(sp.position());
        Vector3 localNormal = m_transform.inverseTransformNormal(sp.shadingNormal());

        SurfaceDifferentials d = m_shape->computeDifferentials(localPoint, 
            localNormal, sp.uv(), sp.triangleIndex());

        return m_transform.transformDifferentials(d, localNormal, sp.shadingNormal());
    }


}