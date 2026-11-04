
#include "integrator/pixelIntegrator.hpp"
#include "config.hpp"
#include "core/random.hpp"
#include "camera/camera.hpp"
#include "core/ray.hpp"
#include "math/vector2.hpp"
#include "math/vector3.hpp"
#include "renderer/progressReporter.hpp"


namespace pathtracer{


    Vector3 PixelIntegrator::renderSample(const Scene& scene, 
        const Camera& camera, int i, int j, int width, int height, 
        bool jitter){

        // We can randomize where on the pixel we shoot
        real offsetX = jitter ? Random::next() : 0.5;
        real offsetY = jitter ? Random::next() : 0.5;

        // Maps to [-1, 1]
        real x = ((i + offsetX) / width) * 2.0 - 1.0;
        real y = 1.0 - ((j + offsetY) / height) * 2.0;

        Ray ray = camera.generateRay(Vector2(x, y));

        // Calls the (subclass's) color function.
        return color(ray, scene);
    }



    std::vector<std::vector<Vector3>> PixelIntegrator::render(
        const Scene& scene, const Camera& camera, int width, 
        int height, ProgressReporter& reporter){

        // This part just renders the image
        int iterations = m_samples;

        std::vector<Vector3> c(width * height);
        // Completed iterations
        std::vector<int> n(width * height, 0);

        bool doJitter = jitter();

        long long total = static_cast<long long>(width) * height * m_samples;
        reporter.start(total, "Rendering");

        for(int k = 0; k < iterations; k++) {

            #pragma omp parallel for schedule(dynamic, 1)
            for(int p = 0; p < width * height; p++) {

                if(g_stopRendering){
                    continue;
                }

                c[p] = c[p] + renderSample(scene, camera, p % width, p / width,
                    width, height, doJitter);

                n[p]++;

                reporter.step();
            }

            if(g_stopRendering){
                break;
            }
        }

        std::vector<std::vector<Vector3>> result(height, std::vector<Vector3>(width));
        for (int j = 0; j < height; j++) {
            for (int i = 0; i < width; i++) {
                int p = j * width + i;
                if (n[p] > 0) {
                    result[j][i] = c[p] / static_cast<real>(n[p]);
                }
            }
        }
        
        return result;
    }

}