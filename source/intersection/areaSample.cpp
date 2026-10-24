
#include "intersection/areaSample.hpp"
#include "intersection/surfaceDifferentials.hpp"

namespace pathtracer{

    AreaSample::AreaSample(
        const Vector3& position,
        const Vector3& geometryNormal,
        const Vector3& shadingNormal,
        const Vector2& uv,        
        const SurfaceDifferentials& differentials,
        const Instance* instance,
        real pdf) : 
        m_pdf(pdf), SurfacePoint(position, geometryNormal, shadingNormal, 
        uv, differentials, instance) {}


    AreaSample::AreaSample(const SurfacePoint& sp, real pdf) : m_pdf(pdf),
        SurfacePoint(sp) {}


    real AreaSample::pdf() const {
        return m_pdf;
    }
    

    void AreaSample::setPdf(real pdf) {
        m_pdf = pdf;
    }

}