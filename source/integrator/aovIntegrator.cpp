
#include "integrator/aovIntegrator.hpp"
#include "intersection/intersection.hpp"
#include "intersection/surfaceDifferentials.hpp"
#include "core/instance.hpp"
#include "texture/texture.hpp"
#include "core/ray.hpp"
#include "core/scene.hpp"
#include "texture/leanMap.hpp"
#include "core/random.hpp"
#include "math/mathUtil.hpp"
#include "sms/smsUtil.hpp"
#include "logger.hpp"

namespace pathtracer{

    AovIntegrator::AovIntegrator(RenderVariable variable) :
        m_variable{variable} {}

    Vector3 AovIntegrator::color(
        const Ray& ray, const Scene& scene) {

        Intersection it = scene.intersect(ray);

        // If background intersection
        if(!it){
            return Vector3::ORIGIN;
        }

        Vector3 color;

        // Otherwise
        switch(m_variable){

        case RenderVariable::SHADING_NORMAL:
            color = (it.shadingNormal() + Vector3(1.0)) * 0.5;
            break;

        case RenderVariable::GEOMETRY_NORMAL:
            color = (it.geometryNormal() + Vector3(1.0)) * 0.5;
            break;

        case RenderVariable::MIPMAP_NORMAL:{

            Vector3 normal = it.shadingNormal();
            
            if(it.instance()->normalTexture()){
                int maxL = it.instance()->normalTexture()->maxMipMapLevel();
                normal = it.instance()->normalTexture()->sampleMipMap(it.uv(), maxL);

                // Must use the geometry frame here since the shading frame is
                // already "infected" by the normal map.
                normal = it.baseFrame().transformDirection(normal);
            }

            color = (normal + Vector3(1.0)) * 0.5;
            break;
        }

        case RenderVariable::LEAN_NORMAL:{
            
            // Uses max level lean normal sampled from gaussian.
            // First the slope is sampled, then transformed to normal.

            Vector3 normal = it.shadingNormal();

            if(it.instance()->leanMap()){

                auto map = it.instance()->leanMap();
                if(!map){
                    LOG_ERROR("No Lean Map defined");
                    throw std::runtime_error("");
                }

                int maxL = map->maxLevel();

                Vector2 mu = LeanMap::mean(map, it.uv(), maxL);
                Matrix2 sigma = LeanMap::covariance(map, it.uv(), maxL);

                // This normal is in tangent/shading space
                Vector2 slope = Gaussian::transform(Random::next2D(), mu, sigma);
                normal = Vector3(-slope[0], -slope[1], 1.0).normalized();

                normal = it.baseFrame().transformDirection(normal);
            }

            color = (normal + Vector3(1.0)) * 0.5;
            break;
        }

        case RenderVariable::DPDU:{
            SurfaceDifferentials d = it.differentials();
            color = (d.dpdu() + Vector3(1.0)) * 0.5;
            break;
        }

        case RenderVariable::DPDV:{
            SurfaceDifferentials d = it.differentials();
            color = (d.dpdv() + Vector3(1.0)) * 0.5;
            break;
        }

        case RenderVariable::DNDU:{
            SurfaceDifferentials d = it.instance()->computeDifferentials(it, false);
            real value = d.dndu().length();
            color = Vector3(value / 1000.0);
            break;
        }

        case RenderVariable::DNDV:{
            SurfaceDifferentials d = it.instance()->computeDifferentials(it, false);
            real v = d.dndv().length();
            color = Vector3(v / 1000.0);
            break;
        }

        case RenderVariable::S:{
            color = (it.s() + Vector3(1.0)) * 0.5;
            break;
        }

        case RenderVariable::DSDU:{
            SurfaceDifferentials sd = it.instance()->computeDifferentials(it);
            auto d = SmsUtil::computeFrameDifferentials(it.shadingNormal(), it.s(), sd);
            color = (d.dsdu + Vector3(1.0)) * 0.5;
            break;
        }

        case RenderVariable::DSDV:{
            SurfaceDifferentials sd = it.instance()->computeDifferentials(it);
            auto d = SmsUtil::computeFrameDifferentials(it.shadingNormal(), it.s(), sd);
            color = (d.dsdv + Vector3(1.0)) * 0.5;
            break;
        }

        case RenderVariable::DTDU:{
            SurfaceDifferentials sd = it.instance()->computeDifferentials(it);
            auto d = SmsUtil::computeFrameDifferentials(it.shadingNormal(), it.s(), sd);
            color = (d.dtdu + Vector3(1.0)) * 0.5;
            break;
        }

        case RenderVariable::DTDV:{
            SurfaceDifferentials sd = it.instance()->computeDifferentials(it);
            auto d = SmsUtil::computeFrameDifferentials(it.shadingNormal(), it.s(), sd);
            color = (d.dtdv + Vector3(1.0)) * 0.5;
            break;
        }

        default:
            color = Vector3::ORIGIN;
        }

        return color;
    }

}