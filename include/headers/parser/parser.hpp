
#ifndef PATH_TRACER_PARSER_HPP
#define PATH_TRACER_PARSER_HPP

#include <string>
#include <vector>

namespace pathtracer{

    class Renderer;
    class Scene;
    class Vector3;

    class Parser{

    public:

        static std::tuple<Scene*, Renderer*, 
            std::vector<std::vector<Vector3>>> parse(const std::string& filename);

    };

}

#endif