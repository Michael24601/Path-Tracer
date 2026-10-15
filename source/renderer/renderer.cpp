
#include "renderer/renderer.hpp"
#include <omp.h>
#include "camera/camera.hpp"
#include "core/ray.hpp"
#include "core/scene.hpp"
#include "integrator/integrator.hpp"
#include "math/vector2.hpp"
#include "math/vector3.hpp"
#include "logger.hpp"
#include <iostream>

namespace pathtracer {

    Renderer::Renderer(int width, int height, const Camera* camera, 
        const Scene* scene, Integrator* integrator, int messageCount) : 
        m_camera{camera},
        m_scene{scene},
        m_integrator{integrator},
        m_width{width},
        m_height{height},
        m_messageCount{messageCount},
        m_pathCounter{0}{}

    // By default, the path count is just the number of pixels
    void Renderer::preprocess() {
        m_totalPathCount = static_cast<long long>(m_width) * m_height;
    }


    std::vector<std::vector<Vector3>> Renderer::render() {
        // Calls preprocess
        preprocess();

        return renderAllPixels();
    }


    std::vector<std::vector<Vector3>> Renderer::renderAllPixels() {

        // This part just renders the image
        int width = m_width;
        int height = m_height;

        std::vector<std::vector<Vector3>> c(height, std::vector<Vector3>(width));

        // This is for multithreading
        #pragma omp parallel for schedule(dynamic, 1)
        for(int j = 0; j < height; j++) {
            for(int i = 0; i < width; i++) {
                c[j][i] = renderPixel(i, j);
            }
        }

        return c;
    }

    // By default, shoots one ray in center of pixel
    Vector3 Renderer::renderPixel(int i, int j) {
        // We map these to the center of the pixels,
        // such that the image ranged between (-1, -1) and (1, 1).

        real offsetX = 0.5;
        real offsetY = 0.5;

        return shootRay(i, j, offsetX, offsetY);
    }

    // Default version just calls the integrator at midpoint of pixel
    Vector3 Renderer::shootRay(int i, int j, real offsetX, real offsetY) {
        real x = ((i + offsetX) / m_width) * 2.0 - 1.0;
        real y = 1.0 - ((j + offsetY) / m_height) * 2.0;

        Vector2 uv(x, y);

        Ray ray = m_camera->generateRay(uv);

        Vector3 color = m_integrator->color(ray, *m_scene);

        // Updates counter
        long long interval = m_totalPathCount / m_messageCount;
        long long pathCount = ++m_pathCounter;

        
        if(pathCount % interval == 0) {
            LOG_TRACE(
                std::to_string(pathCount) + "/" +
                std::to_string(m_totalPathCount) +
                " paths completed."
            );
        }
        
        return color;
    }
}