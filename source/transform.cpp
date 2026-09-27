
#include "../include/headers/core/transform.hpp"
#include "../include/headers/intersection/surfacePoint.hpp"

namespace pathtracer{


    


    Transform Transform::IDENTITY = Transform( 
        Matrix3(
            Vector3(1.0,0.0,0.0),
            Vector3(0.0,1.0,0.0),
            Vector3(0.0,0.0,1.0)
        ), Vector3(0,0,0)
    );


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
        
}