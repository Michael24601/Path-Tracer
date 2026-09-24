
#include "../include/headers/parser/parser.hpp"
#include "../include/headers/sms/specularManifoldSampling.hpp"

using namespace pathtracer;


std::atomic<long long> PathTracerGuided::n1 = 0;
std::atomic<long long> PathTracerGuided::n2 = 0;


int main(int argc, char* argv[]) {

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
        std::cout << "conv - projFailNoHit - projFailDiffInstance - maxIt: " << 
            p->sms.m_converged.load() << " " << p->sms.m_projectionFailedNoHit.load() 
            << " " << p->sms.m_projectionFailedDiffInstance.load() 
            << " " << p->sms.m_maxIt.load() << "\n";
    }

    /*

    Sphere* s = new Sphere();
    MirrorBsdf* bsdf = new MirrorBsdf(0.95);
    Instance* inst = new Instance(s, nullptr, nullptr, bsdf, nullptr,
        Transform::IDENTITY);

    Scene* scene = new Scene(std::vector<Instance*>{inst}, std::vector<Light*>{});

    Vector3 x0 = Vector3(0, -3, 0);
    Vector3 x2 = Vector3(0, 0, 3);

    SpecularManifoldSampling sms(scene);

    LOG_INFO("ONE");

    SmsSample sample = sms.samplePath(x0, inst, bsdf, x2, scene);
    if(sample.isConverged()){

        LOG_INFO("Converged point: " + sample.finalPoint().toString());

        // Then we shoot ray from x0 to converged point and check if it
        // samples same direction.

        Vector3 dir = (sample.finalPoint() - x0).normalized();
        Ray ray(x0 + dir * SHADOW_EPSILON, dir);
        Intersection it = scene->intersect(ray);

        if(it){
            Vector3 wo = (x0 - it.position()).normalized();
            Vector3 wi = (x2 - it.position()).normalized();

            Vector3 reflected = ShadingSpace::reflect(wo, it.shadingNormal());

            LOG_INFO("Point: " + it.position().toString());
            LOG_INFO("wo: " + wo.toString());
            LOG_INFO("wi: " + wi.toString());
            LOG_INFO("reflected: " + reflected.toString());
        }

    }
    
    */

    /*
    

    Sphere* sphere = new Sphere();

    Vector2 uv(0.25, 0.35);
    real h = 1e-5;

    Vector3 p_um = sphere->getPosition(Vector2(uv.x() - h, uv.y()), 0);
    Vector3 p_up = sphere->getPosition(Vector2(uv.x() + h, uv.y()), 0);

    Vector3 p_vm = sphere->getPosition(Vector2(uv.x(), uv.y() - h), 0);
    Vector3 p_vp = sphere->getPosition(Vector2(uv.x(), uv.y() + h), 0);

    Vector3 numericalDpdu = (p_up - p_um) / (2.0 * h);
    Vector3 numericalDpdv = (p_vp - p_vm) / (2.0 * h);

    Vector3 p = sphere->getPosition(uv, 0);
    Vector3 n = p.normalized();

    SurfaceDifferentials d = sphere->computeDifferentials(p, n, uv, 0);
    
    LOG_INFO("numerical dpdu = " + numericalDpdu.toString());
    LOG_INFO("analytic  dpdu = " + d.dpdu().toString());

    LOG_INFO("numerical dpdv = " + numericalDpdv.toString());
    LOG_INFO("analytic  dpdv = " + d.dpdv().toString());

    Vector3 n_um = sphere->getPosition(
    Vector2(uv.x() - h, uv.y()), 0).normalized();

    Vector3 n_up = sphere->getPosition(
        Vector2(uv.x() + h, uv.y()), 0).normalized();

    Vector3 n_vm = sphere->getPosition(
        Vector2(uv.x(), uv.y() - h), 0).normalized();

    Vector3 n_vp = sphere->getPosition(
        Vector2(uv.x(), uv.y() + h), 0).normalized();

    Vector3 numericalDndu = (n_up - n_um) / (2.0 * h);
    Vector3 numericalDndv = (n_vp - n_vm) / (2.0 * h);

    LOG_INFO("numerical dndu = " + numericalDndu.toString());
    LOG_INFO("analytic   dndu = " + d.dndu().toString());

    LOG_INFO("numerical dndv = " + numericalDndv.toString());
    LOG_INFO("analytic   dndv = " + d.dndv().toString());

    Triangle triangle(
        Vector3(-1.5,  0.7,  2.0),
        Vector3( 2.3, -1.1,  0.4),
        Vector3( 0.6,  3.2, -1.7),

        Vector3( 0.3, -0.8,  0.5),
        Vector3(-0.6,  0.4,  0.9),
        Vector3( 0.8,  0.7, -0.2),

        Vector2(-0.4,  0.2),
        Vector2( 1.7, -0.3),
        Vector2( 0.5,  2.1)
    );

    Matrix3 matrix(
        Vector3(2.0,  0.3, -0.2),
        Vector3(0.4,  1.4,  0.5),
        Vector3(-0.1,  0.2,  0.8)
    );

    Instance* inst = new Instance(
        &triangle,
        nullptr, nullptr, nullptr, nullptr,
        Transform(
            matrix,
            Vector3(-2.3, 4.1, 1.7)
        )
    );

    uv = Vector2(0.35, 0.65);

    Vector3 position = inst->getPosition(uv, 0);
    SurfacePoint s = inst->evaluateAreaSample(SurfaceSample{position, 0});

    Vector3 geometryNormal = s.geometryNormal();

    Vector3 shadingNormal = s.shadingNormal();

    d = inst->computeDifferentials(s);

    h = 1e-5;

    p_um = inst->getPosition(Vector2(uv.x() - h, uv.y()), 0);

    p_up = inst->getPosition(Vector2(uv.x() + h, uv.y()), 0);

    p_vm = inst->getPosition(Vector2(uv.x(), uv.y() - h), 0);

    p_vp = inst->getPosition(Vector2(uv.x(), uv.y() + h), 0);

    numericalDpdu =
        (p_up - p_um) / (2.0 * h);

    numericalDpdv =
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

    n_um = getShadingNormal(Vector2(uv.x() - h, uv.y()));
    n_up = getShadingNormal(Vector2(uv.x() + h, uv.y()));

    n_vm = getShadingNormal(Vector2(uv.x(), uv.y() - h));
    n_vp = getShadingNormal(Vector2(uv.x(), uv.y() + h));

    numericalDndu =
        (n_up - n_um) / (2.0 * h);

    numericalDndv =
        (n_vp - n_vm) / (2.0 * h);

    LOG_INFO("numerical dndu = " + numericalDndu.toString());
    LOG_INFO("analytic   dndu = " + d.dndu().toString());

    LOG_INFO("numerical dndv = " + numericalDndv.toString());
    LOG_INFO("analytic   dndv = " + d.dndv().toString());

    LOG_INFO("n      = " + getShadingNormal(uv).toString());
    LOG_INFO("n_um   = " + n_um.toString());
    LOG_INFO("n_up   = " + n_up.toString());
    LOG_INFO("n_vm   = " + n_vm.toString());
    LOG_INFO("n_vp   = " + n_vp.toString());

    */

    /*


    Triangle triangle(
        Vector3(-1.5,  0.7,  2.0),
        Vector3( 2.3, -1.1,  0.4),
        Vector3( 0.6,  3.2, -1.7),

        Vector3( 0.3, -0.8,  0.5),
        Vector3(-0.6,  0.4,  0.9),
        Vector3( 0.8,  0.7, -0.2),

        Vector2(-0.4,  0.2),
        Vector2( 1.7, -0.3),
        Vector2( 0.5,  2.1)
    );

    Matrix3 matrix(
        Vector3(2.0,  0.3, -0.2),
        Vector3(0.4,  1.4,  0.5),
        Vector3(-0.1,  0.2,  0.8)
    );

    Instance* inst = new Instance(
        &triangle,
        nullptr, nullptr, nullptr, nullptr,
        Transform(
            matrix,
            Vector3(-2.3, 4.1, 1.7)
        )
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
    Vector3 p0 = inst->getPosition(uv, 0);
    Vector3 x0 = p0 + Vector3(2.0, 1.0, 3.0);
    Vector3 x2 = p0 + Vector3(-3.0, 2.0, 4.0);


    auto getSurfacePoint = [&](const Vector2& uv) {
        Vector3 position = inst->getPosition(uv, 0);

        return inst->evaluateAreaSample(
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

    */
    
    return 0;
}