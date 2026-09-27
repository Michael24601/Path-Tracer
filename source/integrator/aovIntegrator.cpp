
#include "integrator/aovIntegrator.hpp"
#include "intersection/intersection.hpp"
#include "intersection/surfaceDifferentials.hpp"
#include "core/instance.hpp"
#include "core/ray.hpp"
#include "core/scene.hpp"
#include "sms/smsUtil.hpp"

namespace pathtracer{

    AovIntegrator::AovIntegrator(RenderVariable variable) :
        m_variable{variable} {}

    Vector3 AovIntegrator::color(
        const Ray& ray, const Scene& scene) {

        if(m_variable == RenderVariable::DIRECTION){
            return (ray.direction() + Vector3(1.0)) * 0.5;
        }

        Intersection it = scene.intersect(ray);

        // If background intersection
        if(!it){
            return Vector3::ORIGIN;
        }

        Vector3 color;

        // Otherwise
        switch(m_variable){
        case RenderVariable::ALBEDO:
            color = Vector3(1, 0, 0);
            break;

        case RenderVariable::NORMAL:
            color = (it.shadingNormal() + Vector3(1.0)) * 0.5;
            break;

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