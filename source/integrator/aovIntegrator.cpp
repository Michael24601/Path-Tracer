
#include "integrator/aovIntegrator.hpp"
#include "intersection/intersection.hpp"
#include "intersection/surfaceDifferentials.hpp"
#include "core/instance.hpp"
#include "core/ray.hpp"
#include "core/scene.hpp"
#include "texture/leanMap.hpp"
#include "core/random.hpp"
#include "math/mathUtil.hpp"
#include "sms/smsUtil.hpp"

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

        case RenderVariable::LEAN_NORMAL:{
            // Uses max level lean normal sampled from gaussian.
            // First the slope is sampled, then transformed to normal.
            int maxL = it.instance()->leanMap()->maxLevel(it.instance()->normalTexture());
            Vector2 mu = it.instance()->leanMap()->mean(it.uv(), maxL);
            Matrix2 sigma = it.instance()->leanMap()->covariance(it.uv(), maxL);

            // This normal is in tangent/shading space
            Vector2 slope = Gaussian::transform(Random::next2D(), mu, sigma);
            Vector3 normal = Vector3(-slope[0], -slope[1], 1.0).normalized();

            normal = it.shadingFrame().transformDirection(normal);
            color = (normal + Vector3(1.0)) * 0.5;
            break;
        }

        case RenderVariable::DPDU:{
            SurfaceDifferentials d =
                it.instance()->computeDifferentials(it);
            color = (d.dpdu() + Vector3(1.0)) * 0.5;
            break;
        }

        case RenderVariable::DPDV:{
            SurfaceDifferentials d =
                it.instance()->computeDifferentials(it);
            color = (d.dpdv() + Vector3(1.0)) * 0.5;
            break;
        }

        case RenderVariable::DNDU:{
            SurfaceDifferentials d =
                it.instance()->computeDifferentials(it);
            real value = d.dndu().length();
            color = Vector3(value / 1000.0);
            break;
        }

        case RenderVariable::DNDV:{
            SurfaceDifferentials d =
                it.instance()->computeDifferentials(it);
            real v = d.dndv().length();
            color = Vector3(v);
            break;
        }

        case RenderVariable::S:{
            SurfaceDifferentials d =
                it.instance()->computeDifferentials(it);
            real v = d.dndv().length();
            color = (d.s() + Vector3(1.0)) * 0.5;
            break;
        }

        case RenderVariable::DSDU:{
            SurfaceDifferentials sd =
                it.instance()->computeDifferentials(it);
            auto d = SmsUtil::computeFrameDifferentials(
                it.shadingNormal(), sd);
            color = (d.dsdu + Vector3(1.0)) * 0.5;
            break;
        }

        case RenderVariable::DSDV:{
            SurfaceDifferentials sd =
                it.instance()->computeDifferentials(it);
            auto d = SmsUtil::computeFrameDifferentials(
                it.shadingNormal(), sd);
            color = (d.dsdv + Vector3(1.0)) * 0.5;
            break;
        }

        case RenderVariable::DTDU:{
            SurfaceDifferentials sd =
                it.instance()->computeDifferentials(it);
            auto d = SmsUtil::computeFrameDifferentials(
                it.shadingNormal(), sd);
            color = (d.dtdu + Vector3(1.0)) * 0.5;
            break;
        }

        case RenderVariable::DTDV:{
            SurfaceDifferentials sd =
                it.instance()->computeDifferentials(it);
            auto d = SmsUtil::computeFrameDifferentials(
                it.shadingNormal(), sd);
            color = (d.dtdv + Vector3(1.0)) * 0.5;
            break;
        }

        default:
            color = Vector3::ORIGIN;
        }

        return color;
    }

}