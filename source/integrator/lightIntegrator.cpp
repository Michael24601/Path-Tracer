
#include "integrator/lightIntegrator.hpp"
#include "math/vector3.hpp"
#include "camera/camera.hpp"
#include "math/vector2.hpp"
#include "math/vector3.hpp"
#include "renderer/progressReporter.hpp"
#include "core/scene.hpp"
#include "logger.hpp"
#include <omp.h>

namespace pathtracer{


    void LightIntegrator::splat(const Vector3& contribution, 
        const CameraConnection& connection, 
        std::vector<std::vector<Vector3>>& image){

        // The image resolution
        int w = image[0].size();
        int h = image.size();

        // The pixel coordinates (here x and y are the normalized coordinates
        // on the camera plane)
        int i = std::min(static_cast<int>(
            (connection.point.x() + 1) * 0.5 * w), w - 1);
        int j = std::min(static_cast<int>(
            (1 - connection.point.y()) * 0.5 * h), h - 1);

        image[j][i] = image[j][i] + contribution;
    }



    std::vector<std::vector<Vector3>> LightIntegrator::render(
        const Scene& scene, const Camera& camera, int width,
        int height, ProgressReporter& reporter){

        int threadCount = omp_get_max_threads();

        // We store an image for every single thread
        std::vector<std::vector<std::vector<Vector3>>> images(
            threadCount, std::vector<std::vector<Vector3>>(
                height, std::vector<Vector3>(width, Vector3(0.0))));

        reporter.start(m_lightPaths, "Rendering");

        long long completedPaths = 0;

        #pragma omp parallel
        {
            int thread = omp_get_thread_num();

            #pragma omp for schedule(dynamic, 1)
            for(long long p = 0; p < m_lightPaths; p++) {

                if(g_stopRendering){
                    continue;
                }

                tracePath(scene, camera, images[thread]);

                #pragma omp atomic
                completedPaths++;

                reporter.step();
            }
        }

        std::vector<std::vector<Vector3>> result(
            height, std::vector<Vector3>(width, Vector3(0.0)));

        for(int t = 0; t < threadCount; t++) {
            for(int j = 0; j < height; j++) {
                for(int i = 0; i < width; i++) {
                    result[j][i] = result[j][i] + images[t][j][i];
                }
            }
        }

        // We have to normalized brightness by path count. If that
        // count is 0, we don't normalize by anything.
        // The normalization MUST include the image dimensions as well.
        real normalization = completedPaths > 0
            ? static_cast<real>(width) * static_cast<real>(height)
                / static_cast<real>(completedPaths)
            : 0.0;

        for(int j = 0; j < height; j++) {
            for(int i = 0; i < width; i++) {
                result[j][i] = result[j][i] * normalization;
            }
        }

        return result;
    }

}