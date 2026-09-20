
#include "../include/headers/parser/parser.hpp"
#include "../include/headers/sms/specularManifoldSampling.hpp"

using namespace pathtracer;


std::atomic<long long> PathTracerGuided::n1 = 0;
std::atomic<long long> PathTracerGuided::n2 = 0;


int main(int argc, char* argv[]) {

    /*

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


    */


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
    
    
    return 0;
}