
#include "include/headers/core/instance.hpp"
#include "include/headers/intersection/areaSample.hpp"

namespace pathtracer{
        
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


    AreaSample Instance::evaluateAreaSample(const Vector3& point) const{
        // First we transform the point to local
        // coordinates.
        Vector3 localPoint = m_transform.inverseTransform(point);

        // Then we evaluate the area sample.
        // Both the position and the pdf are in the shape's
        // local coordinates, so we can transform them.
        AreaSample sample = m_shape->evaluateAreaSample(localPoint);

        // The transform matrix is used directly since we don't
        // want it normalized yet.
        Transform m = m_transform.inverseTranspose();
        Vector3 normalWorld = m.transform(sample.shadingNormal());

        // Transforming a local pdf to a world pdf
        real worldPdf = sample.pdf() / (normalWorld.length() 
            * std::abs(m_transform.determinant()));

        // We then transform the whole sample
        SurfacePoint newSample = m_transform.transformSurfacePoint(sample);

        // The total pdf is just the pdf of choosing the point
        // multilplied by the pdf of choosing the shape.
        return AreaSample(newSample, worldPdf);   
    }
    
}