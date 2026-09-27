
#include "../include/headers/parser/parser.hpp"
#include "../include/headers/sms/specularManifoldSampling.hpp"

using namespace pathtracer;


std::atomic<long long> PathTracerGuided::n1 = 0;
std::atomic<long long> PathTracerGuided::n2 = 0;


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

    std::cout << PathTracerGuided::n1.load(std::memory_order_relaxed) 
        << " " << PathTracerGuided::n2.load(std::memory_order_relaxed) << "\n";

    PathTracerSms* p = dynamic_cast<PathTracerSms*>(renderer->m_integrator);
    if(p){
        std::cout << "conv - maxIt - sameFound - sameNotFound - notVis: " << 
            p->sms.m_converged.load()
            << " " << p->sms.m_maxIt.load()
            << " " << p->sms.m_sameFound.load() 
            << " " << p->sms.m_sameNotFound.load()
            << " " << p->sms.m_notVis.load()  << "\n";
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


    // Fixed previous and next vertices
    Vector3 x0 = Vector3(0.8, 0.0, 0.2);
    Vector2 uv2(0.5, 0.3);
    Vector3 x2 = light->getPosition(uv2, 0);


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
    SurfacePoint p2 = getSurfacePoint2(uv2);

    Vector3 wo = (x0 - p.position()).normalized();
    Vector3 wi = (x2 - p.position()).normalized();

    Vector3 ha = SmsUtil::halfVector(
        wo, p, wi, eta, reflection
    );

    d = inst->computeDifferentials(p);

    HalfVectorDifferentials hd = SmsUtil::computeHalfVectorDifferentials(
        x0, x2,
        p,
        wo, wi,
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


    // Constraint


    auto getConstraint = [&](const Vector2& uv) {
        SurfacePoint p = getSurfacePoint(uv);

        Vector3 wo = (x0 - p.position()).normalized();
        Vector3 wi = (x2 - p.position()).normalized();

        Vector3 h = SmsUtil::halfVector(
            wo, p, wi, eta, reflection
        ).normalized();

        SurfaceDifferentials d = inst->computeDifferentials(p);

        return SmsUtil::specularConstraint(p, h, d);
    };


    hstep = 1e-5;

    Vector2 c_um = getConstraint(
        Vector2(uv.x() - hstep, uv.y())
    );

    Vector2 c_up = getConstraint(
        Vector2(uv.x() + hstep, uv.y())
    );

    Vector2 c_vm = getConstraint(
        Vector2(uv.x(), uv.y() - hstep)
    );

    Vector2 c_vp = getConstraint(
        Vector2(uv.x(), uv.y() + hstep)
    );

    Vector2 numericalDu = (c_up - c_um) / (2.0 * hstep);

    Vector2 numericalDv = (c_vp - c_vm) / (2.0 * hstep);


    p = getSurfacePoint(uv);
    wo = (x0 - p.position()).normalized();
    wi = (x2 - p.position()).normalized();

    Vector3 half = SmsUtil::halfVector(
        wo, p, wi, eta, reflection
    );

    d = inst->computeDifferentials(p);
    SurfaceDifferentials d2 = light->computeDifferentials(p2);

    Matrix2 J = SmsUtil::computeConstraintJacobian(
        x0, x2,
        p,
        wo, wi,
        half,
        d,
        reflection,
        eta
    );


    LOG_INFO("numerical dC/du = " + numericalDu.toString());
    LOG_INFO("analytic   dC/du = " +
        Vector2(J(0,0), J(1,0)).toString());

    LOG_INFO("numerical dC/dv = " + numericalDv.toString());
    LOG_INFO("analytic   dC/dv = " +
        Vector2(J(0,1), J(1,1)).toString());


    // dc1/dx1 and dc1/dx2

    auto getConstraint2 = [&](const Vector3& x1, const Vector3& x2) {
        SurfacePoint p = getSurfacePoint(uv);

        // Replace the position of p with x1 for the derivative test
        p.setPosition(x1);

        Vector3 wo = (x0 - x1).normalized();
        Vector3 wi = (x2 - x1).normalized();

        Vector3 h = SmsUtil::halfVector(
            wo, p, wi, eta, reflection
        ).normalized();

        SurfaceDifferentials d = inst->computeDifferentials(p);

        return SmsUtil::specularConstraint(p, h, d);
    };

    hstep = 1e-5;

    Vector3 x2_um = light->getPosition(Vector2(uv2.x() - hstep, uv2.y()), 0);
    Vector3 x2_up = light->getPosition(Vector2(uv2.x() + hstep, uv2.y()), 0);

    Vector3 x2_vm = light->getPosition(Vector2(uv2.x(), uv2.y() - hstep), 0);
    Vector3 x2_vp = light->getPosition(Vector2(uv2.x(), uv2.y() + hstep), 0);

    Vector2 c_x2_um = getConstraint2(p.position(), x2_um);
    Vector2 c_x2_up = getConstraint2(p.position(), x2_up);

    Vector2 c_x2_vm = getConstraint2(p.position(), x2_vm);
    Vector2 c_x2_vp = getConstraint2(p.position(), x2_vp);

    Vector2 numericalDcDx2_u =
        (c_x2_up - c_x2_um) / (2.0 * hstep);

    Vector2 numericalDcDx2_v =
        (c_x2_vp - c_x2_vm) / (2.0 * hstep);

    
    auto [dc1_dx1, dc1_dx2] =
        SmsUtil::computeConstraintJacobians(
            x0, p, p2, d, d2, ha, eta, reflection
        );

    LOG_INFO("numerical dC/dx2_u = " +
        numericalDcDx2_u.toString());

    LOG_INFO("analytic   dC/dx2_u = " +
        Vector2(
            dc1_dx2(0, 0),
            dc1_dx2(1, 0)
        ).toString());

    LOG_INFO("numerical dC/dx2_v = " +
        numericalDcDx2_v.toString());

    LOG_INFO("analytic   dC/dx2_v = " +
        Vector2(
            dc1_dx2(0, 1),
            dc1_dx2(1, 1)
        ).toString());

    */
        
    return 0;
}