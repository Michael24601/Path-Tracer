
#ifndef PATH_TRACER_PARSER_HPP
#define PATH_TRACER_PARSER_HPP

#include <string>
#include <memory>

namespace pathtracer{

    class Renderer;
    class Scene;

    class Parser{

    public:

        static std::pair<std::unique_ptr<Scene>, std::unique_ptr<Renderer>> 
            parse(const std::string& filename);

    };

}

#endif