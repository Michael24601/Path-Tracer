
#include "include/headers/core/transform.hpp"
#include "include/headers/intersection/surfacePoint.hpp"

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
            
        return res;
    }
        
}