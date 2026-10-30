#include "core/instance.hpp"
#include "shapes/shape.hpp"
#include "texture/texture.hpp"
#include "bsdf/bsdf.hpp"
#include "emission/emission.hpp"
#include "bvh/axisAlignedBox.hpp"
#include "math/vector2.hpp"
#include "math/vector3.hpp"
#include "texture/leanMap.hpp"
#include "intersection/areaSample.hpp"
#include "intersection/surfaceDifferentials.hpp"
#include "sms/smsUtil.hpp"
#include "math/constants.hpp"
#include "intersection/Intersection.hpp"
#include "intersection/Hit.hpp"
#include "core/ray.hpp"
#include "logger.hpp"

namespace pathtracer{


    void Instance::setInScene(bool inScene) {
        m_inScene = inScene;
    }


    Instance::Instance(const Shape* shape, const Texture* alpha, 
        const Texture* normal, const MipMap* leanMap, const Bsdf* bsdf, 
        const Emission* emission, const Transform& transform) : 
        m_shape(shape), 
        m_alpha(alpha), 
        m_normal(normal),
        m_bsdf(bsdf), 
        m_emission(emission),
        m_transform(transform), 
        m_leanMap(leanMap),
        m_light{nullptr}, 
        m_inScene{false},
        m_isCausticReceiver{false}{

        // Cached
        m_determinant = m_transform.determinant();
        m_inverse = m_transform.inverse();
        m_inverseTranspose = m_transform.inverseTranspose();
    }


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


    const MipMap* const Instance::leanMap() const {
        return m_leanMap;
    }


    const Transform& Instance::transform() const {
        return m_transform;
    }


    const Transform& Instance::inverse() const {
        return m_inverse;
    }


    const Shape* Instance::shape() const {
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
        Vector3 normalWorld = m_inverseTranspose.transform(sample.shadingNormal());

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
        areaSample.computeBaseFrame();

        // We can also do normal mapping here
        if(m_normal != nullptr){
            normalMap(areaSample);
        }

        return areaSample;
    }


    AreaSample Instance::evaluateAreaSample(const SurfaceSample& surPoint) const{
        // First we transform the point to local
        // coordinates.
        Vector3 localPoint = m_inverse.transform(surPoint.point);
        SurfaceSample localSample{localPoint, surPoint.triangleIndex};

        // Then we evaluate the area sample.
        // Both the position and the pdf are in the shape's
        // local coordinates, so we can transform them.
        AreaSample sample = m_shape->evaluateAreaSample(localSample);

        // The transform matrix is used directly since we don't
        // want it normalized yet.
        Vector3 normalWorld = m_inverseTranspose.transform(sample.shadingNormal());

        // Transforming a local pdf to a world pdf
        real worldPdf = sample.pdf() / (normalWorld.length() 
            * std::abs(m_transform.determinant()));

        // We then transform the whole sample
        SurfacePoint newSample = m_transform.transformSurfacePoint(sample);

        newSample.setInstance(this);
        newSample.computeShadingFrame();
        newSample.computeBaseFrame();

        // We can also do normal mapping here
        if(m_normal != nullptr){
            normalMap(newSample);
        }

        // The total pdf is just the pdf of choosing the point
        // multilplied by the pdf of choosing the shape.
        return AreaSample(newSample, worldPdf);   
    }


    Intersection Instance::constructIntersection(const Hit& hit,
        const Ray& ray) const{

        // First we get the point in local scape of the intersection.
        // The ray and hit are world coordinates.
        Vector3 point = ray.at(hit.t);
        // Then we transform to local coordinates
        point = m_inverse.transform(point);

        SurfacePoint sp = m_shape->getSurfacePoint(point, hit);
        sp = m_transform.transformSurfacePoint(sp);
        sp.setInstance(this);

        sp.computeShadingFrame();
        sp.computeBaseFrame();

        // We can also do normal mapping here
        if(m_normal != nullptr){
            normalMap(sp);
        }
        
        return Intersection(hit.t, sp);
    }


    SurfaceDifferentials Instance::computeDifferentials(
        const SurfacePoint& sp, bool smooth) const {

        const SurfaceDifferentials& d = sp.differentials();

        if (m_normal == nullptr) {
            return d;
        }

        // If we have a normal map, then we need to change dndu and dndv

        Vector3 n = sp.baseNormal();
        Vector3 s = sp.baseFrame().s;
        Vector3 t = sp.baseFrame().t;

        FrameDifferentials fd = SmsUtil::computeFrameDifferentials(n, s, d);

        Vector2 uv = sp.uv();

        Vector3 normal;
        Vector3 dndu(0.0f);
        Vector3 dndv(0.0f);

        if (smooth) {
            normal = m_normal->sampleMipMap(uv, m_normal->maxMipMapLevel()).normalized();
        }
        else {
            normal = m_normal->sample(uv).normalized();
            m_normal->sampleGradient(uv, dndu, dndv);
        }

        Vector3 resU = fd.dsdu * normal.x() + fd.dtdu * normal.y() 
            + d.dndu() * normal.z() + s * dndu.x() + t * dndu.y() + n * dndu.z();

        Vector3 resV = fd.dsdv * normal.x() + fd.dtdv * normal.y() 
            + d.dndv() * normal.z() + s * dndv.x() + t * dndv.y() + n * dndv.z();

        return SurfaceDifferentials(
            d.dpdu(), d.dpdv(),
            resU, resV,
            d.d2pdu2(), d.d2pdudv(), d.d2pdv2()
        );
    }



    void Instance::normalMap(SurfacePoint& it, bool smooth) const {

        if(m_normal != nullptr){

            // It's already mapped to correct range and normal
            Vector3 normal;
            if(!smooth){
                normal = m_normal->sample(it.uv());
            }
            else{
                int maxL = m_normal->maxMipMapLevel();
                normal = m_normal->sampleMipMap(it.uv(), maxL);
            }

            // We need to map normal from shading frame to world coordinates
            // We can't use normal transform since it applies translation
            // (not suitable for directions).
            // We should in theory use transformNormal which was designed
            // for normals, but since shadingFrame is always orthogonal,
            // transformDirection is equivalent, while much cheaper to
            // compute, so we will use that.

            // Also note that if we have already used normal mapping,
            // then it.shadingFrame() already uses the coarse normal map,
            // which will break normal mapping in case we want to
            // smooth it again. So we use the base frame here.
            normal = it.baseFrame().toWorld(normal);

            // Then the frame is updated
            it.setShadingNormal(normal);
            // Frame is recomputed
            it.computeShadingFrame();
        }
    }
        

}