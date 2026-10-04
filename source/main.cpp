
#include "parser/parser.hpp"
#include "logger.hpp"
#include "image/imageIo.hpp"
#include "renderer/renderer.hpp"
#include "integrator/pathTracerSms.hpp"
#include "sms/specularManifoldSampling.hpp"
#include <iostream>
#include <fstream>


using namespace pathtracer;


int main(int argc, char* argv[]) {

    
    std::ofstream logFile("file.log");
    Logger::setOutput(logFile);
    Logger::setMaxTimeInterval(0.00001);


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

    // Opens image when done
    std::string command = "start \"\" \"" + output + "\"";
    system(command.c_str());


    return 0;
}