
#ifndef PATH_TRACER_IMAGE_UTIL_HPP
#define PATH_TRACER_IMAGE_UTIL_HPP

#include "config.hpp"
#include <vector>

namespace pathtracer{

    class Vector3;

    namespace ImageUtil{

        real relativeMse(const std::vector<std::vector<Vector3>>& image,
            const std::vector<std::vector<Vector3>>& reference);


        real mse(const std::vector<std::vector<Vector3>>& image,
            const std::vector<std::vector<Vector3>>& reference);


        Vector3 meanColor(const std::vector<std::vector<Vector3>>& image);

    }

}

#endif