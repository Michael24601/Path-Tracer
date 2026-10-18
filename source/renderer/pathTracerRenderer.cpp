
#include "renderer/pathTracerRenderer.hpp"
#include "core/random.hpp"
#include "core/ray.hpp"
#include "math/vector2.hpp"
#include "math/vector3.hpp"
#include "camera/camera.hpp"
#include "integrator/integrator.hpp"
#include "logger.hpp"

namespace pathtracer{


    PathTracerRenderer::PathTracerRenderer(int width, int height,
        const Camera* camera, const Scene* scene, Integrator* integrator,
        int samples, int messageCount) :
        Renderer(width, height, camera, scene, integrator, messageCount),
        m_samples{samples} {
    }


    void PathTracerRenderer::preprocess(){
        // This time it also includes sample count
        m_totalPathCount = static_cast<long long>(m_width) * m_height * m_samples;
    }


    // The path tracer version uses multiple samples per pixel,
    // and jitters them.
    Vector3 PathTracerRenderer::renderPixel(int i, int j) {

        Vector3 color;

        for(int k = 0; k < m_samples; k++){
            real offsetX = Random::next();
            real offsetY = Random::next();

            color = color + shootRay(i, j, offsetX, offsetY);
        }

        return color / m_samples;
    }


    std::string PathTracerRenderer::toString() const {
        return m_integrator->toString() + " SPP=" 
            + std::to_string(m_samples) + " "
            + std::to_string(m_height) + "x" 
            + std::to_string(m_width) + "px";
    }

}