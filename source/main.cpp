
#include "parser/parser.hpp"
#include "logger.hpp"
#include "image/imageIo.hpp"
#include "renderer/renderer.hpp"
#include "integrator/pathTracerSms.hpp"
#include "sms/specularManifoldSampling.hpp"
#include "core/scene.hpp"
#include <iostream>
#include <fstream>
#include <sstream>

#include "integrator/guidable.hpp"
#include "pathGuiding/quadTree.hpp"
#include "pathGuiding/kdTree.hpp"
#include "pathGuiding/quadTreeUtil.hpp"


using namespace pathtracer;


int main(int argc, char* argv[]) {

    
    std::ofstream logFile("file.log");
    Logger::setOutput(logFile);
    Logger::setMaxTimeInterval(-1.0);


    if(argc != 3 && argc != 2) {
        LOG_ERROR("Invalid arguments");
        return 1;
    }


    std::string input = std::string("data/") + argv[1];
    Renderer* renderer = Parser::parse(input);

    LOG_INFO("Rendering start.");

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    auto image = renderer->render();
    
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();

    LOG_INFO("Rendering end.");
    LOG_INFO("Duration: " + std::to_string(duration) + " milliseconds");

    std::string outputName;

    // Iif output name already exists, we don't parse one from scene
    if (argc == 3){
        outputName = argv[2];
    }
    else{
        outputName = Parser::parseOutputName(input);
        // The output also includes the time it took
        outputName += "_" + std::to_string(duration) + "ms";
    }

    std::string output = "output/" + outputName + ".png";

    int index = 1;

    while (std::ifstream(output).good()) {
        output = "output/" + outputName +
            "[" + std::to_string(index) + "].png";
        index++;
    }

    ImageIo::savePNG(image, output);

    LOG_INFO("Image saved to " + output);


    auto renderTree = [&](Vector3 p, const std::string& name) {
        auto guidable = dynamic_cast<Guidable*>(renderer->m_integrator);
        auto bbox = renderer->m_scene->getBoundingBox();

        if(guidable && guidable->m_guideTree){

            p = (p - bbox.minCorner()) /
                (bbox.maxCorner() - bbox.minCorner());

            auto tree = guidable->m_guideTree->getDTree(p);

            std::stringstream ss;
            ss << static_cast<const void*>(tree);
            LOG_INFO("name: " + ss.str());

            int c = guidable->m_guideTree->getSampleCount(p);
            LOG_INFO("count " + std::to_string(c));

            auto image = QuadTreeUtil::renderQuadTree(tree, 512);

            ImageIo::savePNG(image, "output/" + name + ".png");
        }
    };

    renderTree(Vector3(0, 1.99, 0), "tree0");
    renderTree(Vector3(0.99, 1.99, 0.99), "tree1");
    renderTree(Vector3(0.99, 1.99, -0.99), "tree2");
    renderTree(Vector3(0.99, 0, -0.99), "tree3");
    renderTree(Vector3(0.99, 0, 0.99), "tree4");


    // Opens image when done
    std::string command = "start \"\" \"" + output + "\"";
    system(command.c_str());


    return 0;
}