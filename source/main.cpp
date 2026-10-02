
#include "parser/parser.hpp"
#include "logger.hpp"
#include "image/imageIo.hpp"
#include "renderer/renderer.hpp"
#include "integrator/pathTracerSms.hpp"
#include "sms/specularManifoldSampling.hpp"
#include <iostream>
#include <fstream>




#include "parser/parser.hpp"
#include <fstream>
#include <unordered_map>
#include <nlohmann/json.hpp>
#include "core/instance.hpp"
#include "core/scene.hpp"
#include "core/transform.hpp"
#include "bsdf/bsdf.hpp"
#include "bsdf/diffuseBsdf.hpp"
#include "bsdf/principledBsdf.hpp"
#include "bsdf/mirrorBsdf.hpp"
#include "bsdf/dielectricBsdf.hpp"
#include "camera/perspectiveCamera.hpp"
#include "emission/lambertianEmission.hpp"
#include "image/imageIo.hpp"
#include "integrator/aovIntegrator.hpp"
#include "integrator/pathTracer.hpp"
#include "integrator/pathTracerNee.hpp"
#include "integrator/pathTracerNeeMis.hpp"
#include "integrator/pathTracerGuided.hpp"
#include "integrator/pathTracerNeeGuided.hpp"
#include "integrator/pathTracerSms.hpp"
#include "light/light.hpp"
#include "light/areaLight.hpp"
#include "light/pointLight.hpp"
#include "light/directionalLight.hpp"
#include "logger.hpp"
#include "math/matrix3.hpp"
#include "math/vector2.hpp"
#include "math/vector2i.hpp"
#include "math/vector3.hpp"
#include "renderer/renderer.hpp"
#include "renderer/pathTracerRenderer.hpp"
#include "renderer/pathGuidingRenderer.hpp"
#include "shapes/mesh.hpp"
#include "shapes/sphere.hpp"
#include "shapes/triangle.hpp"
#include "texture/texture.hpp"
#include "texture/color.hpp"
#include "parser/objLoader.hpp"
#include "sms/smsUtil.hpp"
#include "sms/angleDifferenceConstraint.hpp"
#include "sms/halfVectorConstraint.hpp"
#include "shapes/shape.hpp"
#include "intersection/surfacePoint.hpp"
#include "intersection/surfaceDifferentials.hpp"
#include "intersection/areaSample.hpp"
#include "light/lightSample.hpp"
#include "core/instance.hpp"
#include "math/mathUtil.hpp"
#include "math/matrix2.hpp"


using namespace pathtracer;


int main(int argc, char* argv[]) {

    
    std::ofstream logFile("file.log");
    Logger::setOutput(logFile);


    if(argc != 3) {
        LOG_ERROR("Invalid arguments");
        return 1;
    }


    std::string input = std::string("data/") + argv[1];
    Renderer* renderer = Parser::parse(input);

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    auto image = renderer->render();
    
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    LOG_INFO("Duration: " + 
        std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()) + 
        " milliseconds");

    std::string output = std::string("output/") + argv[2];
    ImageIo::savePNG(image, output);

    // Opens image when done
    std::string command = "start \"\" \"" + output + "\"";
    system(command.c_str());


    PathTracerSms* p = dynamic_cast<PathTracerSms*>(renderer->m_integrator);
    if(p){
        std::cout << "conv - maxIt: " << 
            p->sms->m_converged.load() 
            << " " << p->sms->m_maxIt.load() << "\n";
    }


    /*

    Triangle triangle(
        Vector3(-1.020000, 1.990000, 0.990000),
        Vector3(-1.020000, 1.990000, -1.040000),
        Vector3(1.000000, 1.990000, -1.040000),

        Vector2(0,  1),
        Vector2( 0, 0),
        Vector2(1,  0)
    );

    Triangle triangle2(
        Vector3(1.000000, -0.000000, -1.040000),
        Vector3(1.000000, 0.000000, 0.990000),
        Vector3(1.000000, 1.990000, 0.990000),

        Vector2(0,  1),
        Vector2( 0, 0),
        Vector2(1,  0)
    );


    Sphere* sphere = new Sphere();

    Instance* inst = new Instance(
        sphere,
        nullptr, nullptr, nullptr, nullptr,
        Transform::IDENTITY
    );


    Instance* light = new Instance(
        &triangle,
        nullptr, nullptr, nullptr, nullptr,
        Transform::IDENTITY
    );


    Instance* instF = new Instance(
        &triangle2,
        nullptr, nullptr, nullptr, nullptr,
        Transform::IDENTITY
    );


    Vector2 uv = Vector2(0.35, 0.65);

    Vector3 position = inst->getPosition(uv, 0);
    SurfacePoint s = inst->evaluateAreaSample(SurfaceSample{position, 0});

    Vector3 geometryNormal = s.geometryNormal();
    Vector3 shadingNormal = s.shadingNormal();
    Vector3 tangent = s.tangent();

    SurfaceDifferentials d = inst->computeDifferentials(s);
    FrameDifferentials f = SmsUtil::computeFrameDifferentials(shadingNormal, d);

    real h = 1e-5;

    Vector3 p_um = inst->getPosition(Vector2(uv.x() - h, uv.y()), 0);

    Vector3 p_up = inst->getPosition(Vector2(uv.x() + h, uv.y()), 0);

    Vector3 p_vm = inst->getPosition(Vector2(uv.x(), uv.y() - h), 0);

    Vector3 p_vp = inst->getPosition(Vector2(uv.x(), uv.y() + h), 0);

    Vector3 numericalDpdu =
        (p_up - p_um) / (2.0 * h);

    Vector3 numericalDpdv =
        (p_vp - p_vm) / (2.0 * h);

    LOG_INFO("numerical dpdu = " + numericalDpdu.toString());
    LOG_INFO("analytic   dpdu = " + d.dpdu().toString());

    LOG_INFO("numerical dpdv = " + numericalDpdv.toString());
    LOG_INFO("analytic   dpdv = " + d.dpdv().toString());


    auto getShadingNormal = [&](const Vector2& uv) {
        Vector3 p = inst->getPosition(uv, 0);

        return inst->evaluateAreaSample(
            SurfaceSample{p, 0}
        ).shadingNormal();
    };

    Vector3 n_um = getShadingNormal(Vector2(uv.x() - h, uv.y()));
    Vector3 n_up = getShadingNormal(Vector2(uv.x() + h, uv.y()));

    Vector3 n_vm = getShadingNormal(Vector2(uv.x(), uv.y() - h));
    Vector3 n_vp = getShadingNormal(Vector2(uv.x(), uv.y() + h));

    Vector3 numericalDndu =
        (n_up - n_um) / (2.0 * h);

    Vector3 numericalDndv =
        (n_vp - n_vm) / (2.0 * h);

    LOG_INFO("numerical dndu = " + numericalDndu.toString());
    LOG_INFO("analytic   dndu = " + d.dndu().toString());

    LOG_INFO("numerical dndv = " + numericalDndv.toString());
    LOG_INFO("analytic   dndv = " + d.dndv().toString());


    auto getTangent = [&](const Vector2& uv) {
        Vector3 p = inst->getPosition(uv, 0);
        SurfacePoint sp = inst->evaluateAreaSample(
            SurfaceSample{p, 0}
        );

        SurfaceDifferentials d = inst->computeDifferentials(sp);
        return d.s();
    };


    Vector3 s_um = getTangent(Vector2(uv.x() - h, uv.y()));
    Vector3 s_up = getTangent(Vector2(uv.x() + h, uv.y()));

    Vector3 s_vm = getTangent(Vector2(uv.x(), uv.y() - h));
    Vector3 s_vp = getTangent(Vector2(uv.x(), uv.y() + h));

    Vector3 numericalDsdu =
        (s_up - s_um) / (2.0 * h);

    Vector3 numericalDsdv =
        (s_vp - s_vm) / (2.0 * h);

    LOG_INFO("numerical dsdu = " + numericalDsdu.toString());
    LOG_INFO("analytic   dsdu = " + f.dsdu.toString());

    LOG_INFO("numerical dsdv = " + numericalDsdv.toString());
    LOG_INFO("analytic   dsdv = " + f.dsdv.toString());


    auto getBitangent = [&](const Vector2& uv) {
        Vector3 p = inst->getPosition(uv, 0);
        SurfacePoint sp = inst->evaluateAreaSample(
            SurfaceSample{p, 0}
        );

        SurfaceDifferentials d = inst->computeDifferentials(sp);
        return d.t(sp.shadingNormal());
    };

    Vector3 t_um = getBitangent(Vector2(uv.x() - h, uv.y()));
    Vector3 t_up = getBitangent(Vector2(uv.x() + h, uv.y()));

    Vector3 t_vm = getBitangent(Vector2(uv.x(), uv.y() - h));
    Vector3 t_vp = getBitangent(Vector2(uv.x(), uv.y() + h));

    Vector3 numericalDtdu =
        (t_up - t_um) / (2.0 * h);

    Vector3 numericalDtdv =
        (t_vp - t_vm) / (2.0 * h);

    LOG_INFO("numerical dtdu = " + numericalDtdu.toString());
    LOG_INFO("analytic   dtdu = " + f.dtdu.toString());

    LOG_INFO("numerical dtdv = " + numericalDtdv.toString());
    LOG_INFO("analytic   dtdv = " + f.dtdv.toString());


    // Halfvector

    auto getSurfacePoint = [&](const Vector2& uv) {
        Vector3 position = inst->getPosition(uv, 0);

        return inst->evaluateAreaSample(
            SurfaceSample{position, 0}
        );
    };


    auto getSurfacePoint2 = [&](const Vector2& uv) {
        Vector3 position = light->getPosition(uv, 0);

        return light->evaluateAreaSample(
            SurfaceSample{position, 0}
        );
    };


    auto getLightSample = [&](const Vector2& uv) {
        Vector3 position = light->getPosition(uv, 0);

        LightSample s(Vector3(0.0), Vector3(0.0), position, 0.0, 0.0, nullptr);

        return s;
    };


    auto getSurfacePointF = [&](const Vector2& uv) {
        Vector3 position = instF->getPosition(uv, 0);

        return instF->evaluateAreaSample(
            SurfaceSample{position, 0}
        );
    };


    // Fixed previous and next vertices

    Vector2 uvF(0.35, 0.8);
    auto p0 = getSurfacePointF(uvF);
    Vector3 x0 = p0.position();

    Vector2 uv2(0.5, 0.3);
    auto p2 = getLightSample(uv2);
    Vector3 x2 = p2.position();

    auto getHalfVector = [&](const Vector2& uv,
                            const Vector3& x0,
                            const Vector3& x2,
                            real eta,
                            bool reflection) {

        SurfacePoint p = getSurfacePoint(uv);

        Vector3 wo = (x0 - p.position()).normalized();
        Vector3 wi = (x2 - p.position()).normalized();

        return SmsUtil::halfVector(
            wo, p, wi, eta, reflection
        ).normalized();
    };

    real eta = 1.0;
    bool reflection = true;

    SurfacePoint p = getSurfacePoint(uv);

    Vector3 wo = (x0 - p.position()).normalized();
    Vector3 wi = (x2 - p.position()).normalized();

    Vector3 ha = SmsUtil::halfVector(
        wo, p, wi, eta, reflection
    );

    d = inst->computeDifferentials(p);


    HalfVectorDifferentials hd = HalfVectorConstraint::computeHalfVectorDifferentials(
        p0, p2,
        p,
        ha,
        d,
        reflection,
        eta
    );


    real hstep = 1e-5;

    Vector3 h_um = getHalfVector(
        Vector2(uv.x() - hstep, uv.y()),
        x0, x2, eta, reflection
    );

    Vector3 h_up = getHalfVector(
        Vector2(uv.x() + hstep, uv.y()),
        x0, x2, eta, reflection
    );

    Vector3 h_vm = getHalfVector(
        Vector2(uv.x(), uv.y() - hstep),
        x0, x2, eta, reflection
    );

    Vector3 h_vp = getHalfVector(
        Vector2(uv.x(), uv.y() + hstep),
        x0, x2, eta, reflection
    );

    Vector3 numericalDhdu =
        (h_up - h_um) / (2.0 * hstep);

    Vector3 numericalDhdv =
        (h_vp - h_vm) / (2.0 * hstep);

        
    LOG_INFO("numerical dhdu = " + numericalDhdu.toString());
    LOG_INFO("analytic   dhdu = " + hd.dhdu.toString());

    LOG_INFO("numerical dhdv = " + numericalDhdv.toString());
    LOG_INFO("analytic   dhdv = " + hd.dhdv.toString());


    // wo and wi


        // Angle difference - wo / wi differentials

    DirectionDifferentials dwo =
        AngleDifferenceConstraint::computeWoDifferential(
            p0, p, d
    );

    DirectionDifferentials dwi =
        AngleDifferenceConstraint::computeWiDifferential(
            p, p2, d
    );


    hstep = 1e-5;

    auto getWo = [&](const Vector2& uv) {
        SurfacePoint p = getSurfacePoint(uv);

        return (x0 - p.position()).normalized();
    };


    auto getWi = [&](const Vector2& uv) {
        SurfacePoint p = getSurfacePoint(uv);

        return (x2 - p.position()).normalized();
    };


    Vector3 wo_um = getWo(
        Vector2(uv.x() - hstep, uv.y())
    );

    Vector3 wo_up = getWo(
        Vector2(uv.x() + hstep, uv.y())
    );

    Vector3 wo_vm = getWo(
        Vector2(uv.x(), uv.y() - hstep)
    );

    Vector3 wo_vp = getWo(
        Vector2(uv.x(), uv.y() + hstep)
    );


    Vector3 numericalDwoDu =
        (wo_up - wo_um) / (2.0 * hstep);

    Vector3 numericalDwoDv =
        (wo_vp - wo_vm) / (2.0 * hstep);


    Vector3 wi_um = getWi(
        Vector2(uv.x() - hstep, uv.y())
    );

    Vector3 wi_up = getWi(
        Vector2(uv.x() + hstep, uv.y())
    );

    Vector3 wi_vm = getWi(
        Vector2(uv.x(), uv.y() - hstep)
    );

    Vector3 wi_vp = getWi(
        Vector2(uv.x(), uv.y() + hstep)
    );


    Vector3 numericalDwiDu =
        (wi_up - wi_um) / (2.0 * hstep);

    Vector3 numericalDwiDv =
        (wi_vp - wi_vm) / (2.0 * hstep);


    LOG_INFO("numerical dwo/du = " + numericalDwoDu.toString());
    LOG_INFO("analytic   dwo/du = " + dwo.dwdu.toString());

    LOG_INFO("numerical dwo/dv = " + numericalDwoDv.toString());
    LOG_INFO("analytic   dwo/dv = " + dwo.dwdv.toString());

    LOG_INFO("numerical dwi/du = " + numericalDwiDu.toString());
    LOG_INFO("analytic   dwi/du = " + dwi.dwdu.toString());

    LOG_INFO("numerical dwi/dv = " + numericalDwiDv.toString());
    LOG_INFO("analytic   dwi/dv = " + dwi.dwdv.toString());


    // Angle difference - transformed wo differentials

    Vector3 n = p.shadingNormal();
    Vector3 dndu = d.dndu();
    Vector3 dndv = d.dndv();

    if (n.dot(wo) < 0) {
        n = -n;
        dndu = -dndu;
        dndv = -dndv;
    }


    TransformedDirectionDifferentials ds =
        AngleDifferenceConstraint::transformedDirectionDerivatives(
            wo,
            dwo,
            n,
            dndu,
            dndv,
            reflection,
            eta
        );


    auto getTransformedWo = [&](const Vector2& uv) {
        SurfacePoint p = getSurfacePoint(uv);

        Vector3 wo = (x0 - p.position()).normalized();

        Vector3 n = p.shadingNormal();

        if (n.dot(wo) < 0) {
            n = -n;
        }

        return SmsUtil::transformDirection(
            wo,
            n,
            reflection,
            eta
        );
    };


    Vector3 transformedWo_um = getTransformedWo(
        Vector2(uv.x() - hstep, uv.y())
    );

    Vector3 transformedWo_up = getTransformedWo(
        Vector2(uv.x() + hstep, uv.y())
    );

    Vector3 transformedWo_vm = getTransformedWo(
        Vector2(uv.x(), uv.y() - hstep)
    );

    Vector3 transformedWo_vp = getTransformedWo(
        Vector2(uv.x(), uv.y() + hstep)
    );


    Vector3 numericalDsDu =
        (transformedWo_up - transformedWo_um) / (2.0 * hstep);

    Vector3 numericalDsDv =
        (transformedWo_vp - transformedWo_vm) / (2.0 * hstep);


    LOG_INFO("numerical ds/du = " + numericalDsDu.toString());
    LOG_INFO("analytic   ds/du = " + ds.dsdu.toString());

    LOG_INFO("numerical ds/dv = " + numericalDsDv.toString());
    LOG_INFO("analytic   ds/dv = " + ds.dsdv.toString());


    // Angle difference - spherical differentials

    Vector3 transformedWo =
        SmsUtil::transformDirection(
            wo,
            n,
            reflection,
            eta
        );


    SphericalDifferentials dTransformedWo =
        AngleDifferenceConstraint::computerSphericalDifferentials(
            transformedWo,
            ds.dsdu,
            ds.dsdv
        );


    SphericalDifferentials dWi =
        AngleDifferenceConstraint::computerSphericalDifferentials(
            wi,
            dwi.dwdu,
            dwi.dwdv
        );


    auto getSpherical = [](const Vector3& w) {
        return SphericalCoordinates::transform(w);
    };


    Vector2 sphericalWo_um = getSpherical(
        getTransformedWo(Vector2(uv.x() - hstep, uv.y()))
    );

    Vector2 sphericalWo_up = getSpherical(
        getTransformedWo(Vector2(uv.x() + hstep, uv.y()))
    );

    Vector2 sphericalWo_vm = getSpherical(
        getTransformedWo(Vector2(uv.x(), uv.y() - hstep))
    );

    Vector2 sphericalWo_vp = getSpherical(
        getTransformedWo(Vector2(uv.x(), uv.y() + hstep))
    );


    Vector2 numericalDSphericalWoDu =
        (sphericalWo_up - sphericalWo_um) / (2.0 * hstep);

    Vector2 numericalDSphericalWoDv =
        (sphericalWo_vp - sphericalWo_vm) / (2.0 * hstep);


    auto getSphericalWi = [&](const Vector2& uv) {
        return getSpherical(getWi(uv));
    };


    Vector2 sphericalWi_um = getSphericalWi(
        Vector2(uv.x() - hstep, uv.y())
    );

    Vector2 sphericalWi_up = getSphericalWi(
        Vector2(uv.x() + hstep, uv.y())
    );

    Vector2 sphericalWi_vm = getSphericalWi(
        Vector2(uv.x(), uv.y() - hstep)
    );

    Vector2 sphericalWi_vp = getSphericalWi(
        Vector2(uv.x(), uv.y() + hstep)
    );


    Vector2 numericalDSphericalWiDu =
        (sphericalWi_up - sphericalWi_um) / (2.0 * hstep);

    Vector2 numericalDSphericalWiDv =
        (sphericalWi_vp - sphericalWi_vm) / (2.0 * hstep);


    LOG_INFO("numerical transformed dtheta/d u, dphi/d u = " +
        numericalDSphericalWoDu.toString());

    LOG_INFO("analytic   transformed dtheta/d u, dphi/d u = " +
        Vector2(
            dTransformedWo.dtheta_du,
            dTransformedWo.dphi_du
        ).toString());


    LOG_INFO("numerical transformed dtheta/d v, dphi/d v = " +
        numericalDSphericalWoDv.toString());

    LOG_INFO("analytic   transformed dtheta/d v, dphi/d v = " +
        Vector2(
            dTransformedWo.dtheta_dv,
            dTransformedWo.dphi_dv
        ).toString());


    LOG_INFO("numerical wi dtheta/d u, dphi/d u = " +
        numericalDSphericalWiDu.toString());

    LOG_INFO("analytic   wi dtheta/d u, dphi/d u = " +
        Vector2(
            dWi.dtheta_du,
            dWi.dphi_du
        ).toString());


    LOG_INFO("numerical wi dtheta/d v, dphi/d v = " +
        numericalDSphericalWiDv.toString());

    LOG_INFO("analytic   wi dtheta/d v, dphi/d v = " +
        Vector2(
            dWi.dtheta_dv,
            dWi.dphi_dv
        ).toString());


    // Angle difference - constraint Jacobian

    auto getAngleDifferenceConstraint = [&](const Vector2& uv) {
        SurfacePoint p = getSurfacePoint(uv);

        Vector3 wo =
            (x0 - p.position()).normalized();

        Vector3 wi =
            (x2 - p.position()).normalized();

        Vector3 n = p.shadingNormal();

        if (n.dot(wo) < 0) {
            n = -n;
        }

        Vector3 transformedWo =
            SmsUtil::transformDirection(
                wo,
                n,
                reflection,
                eta
            );

        return AngleDifferenceConstraint::specularConstraint(
            transformedWo,
            wi
        );
    };


    Vector2 c_um = getAngleDifferenceConstraint(
        Vector2(uv.x() - hstep, uv.y())
    );

    Vector2 c_up = getAngleDifferenceConstraint(
        Vector2(uv.x() + hstep, uv.y())
    );

    Vector2 c_vm = getAngleDifferenceConstraint(
        Vector2(uv.x(), uv.y() - hstep)
    );

    Vector2 c_vp = getAngleDifferenceConstraint(
        Vector2(uv.x(), uv.y() + hstep)
    );


    Vector2 numericalDCDu =
        (c_up - c_um) / (2.0 * hstep);

    Vector2 numericalDCDv =
        (c_vp - c_vm) / (2.0 * hstep);


    Matrix2 analyticDCdX(
        dWi.dtheta_du - dTransformedWo.dtheta_du,
        dWi.dtheta_dv - dTransformedWo.dtheta_dv,
        dWi.dphi_du - dTransformedWo.dphi_du,
        dWi.dphi_dv - dTransformedWo.dphi_dv
    );


    LOG_INFO("numerical dC/du = " +
        numericalDCDu.toString());

    LOG_INFO("analytic   dC/du = " +
        Vector2(
            analyticDCdX(0, 0),
            analyticDCdX(1, 0)
        ).toString());


    LOG_INFO("numerical dC/dv = " +
        numericalDCDv.toString());

    LOG_INFO("analytic   dC/dv = " +
        Vector2(
            analyticDCdX(0, 1),
            analyticDCdX(1, 1)
        ).toString());

    */

        
    return 0;
}