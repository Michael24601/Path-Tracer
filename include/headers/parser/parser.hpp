
#ifndef PATH_TRACER_PARSER_HPP
#define PATH_TRACER_PARSER_HPP

#include <string>

namespace pathtracer{

    class Renderer;

    class Parser{

    public:

        static Renderer* parse(const std::string& filename);

    };

}

#endif