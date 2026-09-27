
#include "renderer/renderer.hpp"
#include <omp.h>
#include "camera/camera.hpp"
#include "core/ray.hpp"
#include "core/scene.hpp"
#include "integrator/integrator.hpp"
#include "math/vector2.hpp"
#include "math/vector3.hpp"
#include "logger.hpp"

namespace pathtracer{

    Renderer::Renderer(int width, int height, const Camera* camera,
        const Scene* scene, Integrator* integrator) :
        m_camera{camera},
        m_scene{scene},
        m_integrator{integrator},
        m_width{width},
        m_height{height} {}


    std::vector<std::vector<Vector3>> Renderer::render() {

        int width = m_width;
        int height = m_height;

        std::vector<std::vector<Vector3>> c(
            height,
            std::vector<Vector3>(width));

        // We map these to the center of the pixels,
        // such that the image ranged between (-1, -1) and (1, 1).
        
        // This is for multithreading 
        #pragma omp parallel for
        for(int j = 0; j < height; j++){
            for(int i = 0; i < width; i++){
                c[j][i] = renderPixel(i, j);
            }
        }

        return c;
    }


    // Default version just calls the integrator at midpoint of pixel
    Vector3 Renderer::renderPixel(int i, int j) {

        real offsetX = 0.5, offsetY = 0.5;

        real x = ((i + offsetX) / m_width) * 2.0 - 1.0;
        real y = 1.0 - ((j + offsetY) / m_height) * 2.0;
        Vector2 uv(x, y);

        Ray ray = m_camera->generateRay(uv);

        return m_integrator->color(ray, *m_scene);
    }

}