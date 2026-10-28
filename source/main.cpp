
#include "parser/parser.hpp"
#include "logger.hpp"
#include "image/imageIo.hpp"
#include "renderer/renderer.hpp"
#include "integrator/pathTracerNeeSms.hpp"
#include "sms/specularManifoldSampling.hpp"
#include "core/scene.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <algorithm>
#include <windows.h>

#include "integrator/guidable.hpp"
#include "pathGuiding/kdTree.hpp"
#include "pathGuiding/quadTreeUtil.hpp"


using namespace pathtracer;


int main(int argc, char* argv[]) {

    
    std::ofstream logFile("file.log");
    Logger::setOutput(logFile);
    Logger::setMaxTimeInterval(-1.0);
    Logger::setConsoleOutput(true);

    SetConsoleOutputCP(CP_UTF8);
    std::string line;
    for(int i = 0; i < 130; i++) line += "─";
    std::cout << "\n" << line << "\n\n";
    std::cout.flush();

    if(argc != 3 && argc != 2) {
        LOG_ERROR("Invalid arguments");
        return 1;
    }

    try {

        LOG_INFO("Parsing and building scene.");

        std::string input = std::string("data/") + argv[1];
        auto [scene, renderer] = Parser::parse(input);

        LOG_INFO("Rendering start.");

        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        auto image = renderer->render();

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();

        LOG_INFO("Rendering end.");
        LOG_INFO("Duration: " + std::to_string(duration) + " milliseconds");

        std::string outputName;

        // If output name already exists, we don't parse one from scene
        if (argc == 3){
            outputName = argv[2];
        }
        else{
            std::string sceneName = std::filesystem::path(input).stem().string();
            std::string rendererName = renderer->toString();
            std::replace(rendererName.begin(), rendererName.end(), ' ', '_');

            outputName = sceneName + "_" + rendererName;
            // The output also includes the duration
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

        // Opens image when done
        std::string command = "start \"\" \"" + output + "\"";
        system(command.c_str());

    }
    catch(const std::exception& e) {

        if(e.what()[0] != '\0'){
            LOG_ERROR(e.what());
        }

        std::cout << "\n" << line << "\n\n";
        std::cout.flush();

        return 1;
    }

    std::cout << "\n" << line << "\n\n";
    std::cout.flush();

    return 0;
}