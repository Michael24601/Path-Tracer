
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define TINYEXR_IMPLEMENTATION

#include "tinyexr/tinyexr.h"
#include "image/imageIo.hpp"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <limits>
#include <cmath>
#include <stb_image.h>
#include <stb_image_write.h>
#include "logger.hpp"
#include <cstdlib>

namespace pathtracer{

    namespace ImageIo{

        void savePNG(
            const std::vector<std::vector<Vector3>>& image,
            const std::string& filename) {

            int height = image.size();
            if (height == 0) {
                return;
            }

            int width = image[0].size();
            if (width == 0) {
                return;
            }

            std::vector<unsigned char> pixels(width * height * 3);
            
            auto gammaCorrect = [](double v) {
                v = std::clamp(v, 0.0, 1.0);

                if (v <= 0.0031308)
                    return 12.92 * v;

                return 1.055 * std::pow(v, 1.0 / 2.4) - 0.055;
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

        bool loadPng(const std::string& filename,
            std::vector<std::vector<Vector3>>& im) {

            int width, height, channels;

            unsigned char* data = stbi_load(
                filename.c_str(),
                &width,
                &height,
                &channels,
                3
            );

            if (!data){
                return false;
            }

            std::vector<std::vector<Vector3>> image(height,
                std::vector<Vector3>(width));

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

            im = std::move(image);

            return true;
        }


        bool saveEXR(
            const std::vector<std::vector<Vector3>>& image,
            const std::string& filename) {

            int height = static_cast<int>(image.size());
            if (height == 0 || image[0].empty()) {
                return false;
            }

            int width = static_cast<int>(image[0].size());

            std::vector<float> pixels(width * height * 3);

            for (int j = 0; j < height; j++) {
                for (int i = 0; i < width; i++) {
                    const Vector3& c = image[j][i];

                    int index = (j * width + i) * 3;

                    pixels[index]     = static_cast<float>(c.x());
                    pixels[index + 1] = static_cast<float>(c.y());
                    pixels[index + 2] = static_cast<float>(c.z());
                }
            }

            const char* err = nullptr;

            int ret = SaveEXR(
                pixels.data(),
                width,
                height,
                3,
                0,
                filename.c_str(),
                &err
            );

            if (ret != TINYEXR_SUCCESS) {
                if (err) {
                    LOG_ERROR(std::string("Failed to save EXR: ") + err);
                    FreeEXRErrorMessage(err);
                }
                return false;
            }

            return true;
        }


        bool loadEXR(const std::string& filename,
            std::vector<std::vector<Vector3>>& im) {

            float* data = nullptr;
            int width, height;
            const char* err = nullptr;

            int ret = LoadEXR(
                &data,
                &width,
                &height,
                filename.c_str(),
                &err
            );

            if (ret != TINYEXR_SUCCESS) {
                if (err) {
                    LOG_ERROR(std::string("Failed to load EXR: ") + err);
                    FreeEXRErrorMessage(err);
                }
                return false;
            }

            std::vector<std::vector<Vector3>> image(
                height, std::vector<Vector3>(width));

            for (int j = 0; j < height; j++) {
                for (int i = 0; i < width; i++) {

                    int index = (j * width + i) * 4;

                    image[j][i] = Vector3(
                        data[index],
                        data[index + 1],
                        data[index + 2]
                    );
                }
            }

            free(data);

            im = std::move(image);

            return true;
        }

        
    }

}