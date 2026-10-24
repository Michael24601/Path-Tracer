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
#include "logger.hpp"

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
        m_leanMap{nullptr},
        m_isCausticReceiver{false}{
            
        if(m_normal){   
            m_leanMap = new LeanMap(m_normal);
        }
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


    const LeanMap* const Instance::leanMap() const {
        return m_leanMap;
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
        areaSample.computeGeometryFrame();

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
        newSample.computeGeometryFrame();

        // We can also do normal mapping here
        if(m_normal != nullptr){
            normalMap(newSample);
        }

        // The total pdf is just the pdf of choosing the point
        // multilplied by the pdf of choosing the shape.
        return AreaSample(newSample, worldPdf);   
    }


    SurfaceDifferentials Instance::computeDifferentials(const SurfacePoint& sp,
        bool smooth) const{

        // First we will transform the position and normal to local
        // coordinates.
        Vector3 localPoint  = m_transform.inverseTransform(sp.position());
        Vector3 localNormal = m_transform.inverseTransformNormal(sp.baseNormal());

        SurfaceDifferentials d = m_shape->computeDifferentials(localPoint,
            localNormal, sp.uv(), sp.triangleIndex());

        d = m_transform.transformDifferentials(d, localNormal, sp.baseNormal());

        if(m_normal != nullptr){

            const Vector3 n = sp.shadingNormal();
            const Vector2 uv = sp.uv();

            // Map normal value and its change per uv, in the map's own
            // coordinates (not yet transformed)
            Vector3 nm, dndu, dndv;

            auto surfacePointAt = [&](const Vector2& uv) -> SurfacePoint {
                int ind = sp.triangleIndex();
                Vector3 p = m_shape->getPosition(uv, ind);
                return evaluateAreaSample({p, ind});
            };

            auto mappedNormal = [&](const Vector2& uv) -> Vector3 {
                SurfacePoint p = surfacePointAt(uv);

                Vector3 nm;
                if(smooth){
                    nm = m_normal->sampleMipMap(uv, m_normal->maxMipMapLevel());
                }
                else{
                    nm = m_normal->sample(uv);
                }

                return p.geometryFrame().transformDirection(nm).normalized();
            };

            if(smooth){
                dndu = Vector3(0.0);
                dndv = Vector3(0.0);
            }
            else{
                const real du = 1.0 / m_normal->getWidth();
                const real dv = 1.0 / m_normal->getHeight();

                dndu = (mappedNormal(uv + Vector2(du, 0))
                    - mappedNormal(uv - Vector2(du, 0))) / (2 * du);

                dndv = (mappedNormal(uv + Vector2(0, dv))
                    - mappedNormal(uv - Vector2(0, dv))) / (2 * dv);
            }

            d.setDndu(dndu);
            d.setDndv(dndv);

            Vector3 mappedN = mappedNormal(sp.uv());

            // s must be perpendicular to the mapped normal
            d.setS((d.dpdu() - mappedN * mappedN.dot(d.dpdu())).normalized());

        }

        return d;
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
            // smooth it again. So we use the geometry frame here.
            normal = it.geometryFrame().transformDirection(normal);

            // LOG_INFO(normal.toString() + " " + it.geometryNormal().toString());

            // Then the frame is updated
            it.setShadingNormal(normal);

            // Frame is recomputed
            it.computeShadingFrame();
        }
    }
        

}