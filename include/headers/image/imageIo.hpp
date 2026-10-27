
#ifndef PATH_TRACER_IMAGE_IO_HPP
#define PATH_TRACER_IMAGE_IO_HPP

#include <string>
#include <vector>
#include "math/vector3.hpp"

namespace pathtracer{

    namespace ImageIo{

        void savePNG(
            const std::vector<std::vector<Vector3>>& image,
            const std::string& filename);

        bool loadImage(const std::string& filename,
            std::vector<std::vector<Vector3>>& im);

    }

}

#endif