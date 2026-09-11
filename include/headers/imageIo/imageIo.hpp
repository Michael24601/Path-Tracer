
#ifndef PATH_TRACER_IMAGE_IO_HPP
#define PATH_TRACER_IMAGE_IO_HPP

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION

#include <stb_image_write.h>
#include <stb_image.h>
#include "../math/vector3.hpp"

namespace pathtracer{

    class ImageIo{

    public:

        static void savePNG(const std::vector<std::vector<Vector3>>& image,
            const std::string& filename) {

            int height = static_cast<int>(image.size());
            if (height == 0) return;

            int width = static_cast<int>(image[0].size());
            if (width == 0) return;

            std::vector<unsigned char> pixels(width * height * 3);

            auto gammaCorrect = [](double v) {
                v = std::clamp(v, 0.0, 1.0);
                return std::pow(v, 1.0 / 2.2);
            };

            for (int j = 0; j < height; j++) {
                for (int i = 0; i < width; i++) {

                    const Vector3& c = image[j][i];

                    int index = (j * width + i) * 3;

                    pixels[index + 0] = static_cast<unsigned char>(
                        gammaCorrect(c.x()) * 255.0 + 0.5);

                    pixels[index + 1] = static_cast<unsigned char>(
                        gammaCorrect(c.y()) * 255.0 + 0.5);

                    pixels[index + 2] = static_cast<unsigned char>(
                        gammaCorrect(c.z()) * 255.0 + 0.5);
                }
            }

            stbi_write_png(
                filename.c_str(),
                width,
                height,
                3,
                pixels.data(),
                width * 3
            );
        }


        static std::vector<std::vector<Vector3>> loadImage(
            const std::string& filename) {

            int width, height, channels;

            unsigned char* data = stbi_load(
                filename.c_str(),
                &width,
                &height,
                &channels,
                3
            );

            if (!data)
                return {};

            std::vector<std::vector<Vector3>> image(
                height,
                std::vector<Vector3>(width)
            );

            for (int j = 0; j < height; j++) {
                for (int i = 0; i < width; i++) {

                    int index = (j * width + i) * 3;

                    image[j][i] = Vector3(
                        data[index + 0] / 255.0,
                        data[index + 1] / 255.0,
                        data[index + 2] / 255.0
                    );
                }
            }

            stbi_image_free(data);

            return image;
        }
        

    };
}

#endif