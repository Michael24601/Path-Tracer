
#include "../include/headers/parser/parser.hpp"

using namespace pathtracer;

int main(int argc, char* argv[]) {

    if(argc != 3) {
        LOG_ERROR("Invalid arguments");
        return 1;
    }

    std::string input = std::string("data/") + argv[1];
    Renderer* renderer = Parser::parse(input);
    auto image = renderer->render();

    std::string output = std::string("output/") + argv[2];
    ImageIo::savePNG(image, output);

    // Opens image when done
    std::string command = "start \"\" \"" + output + "\"";
    system(command.c_str());

    return 0;
}