
#include "../include/headers/parser/parser.hpp"

using namespace pathtracer;

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

    return 0;
}