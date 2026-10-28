
#ifndef PATH_TRACER_PARSER_HPP
#define PATH_TRACER_PARSER_HPP

#include <string>

namespace pathtracer{

    class Renderer;
    class Scene;

    class Parser{

    public:

        static std::pair<Scene*, Renderer*> parse(const std::string& filename);

    };

}

#endif