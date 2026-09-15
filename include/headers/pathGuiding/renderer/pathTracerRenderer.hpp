
#ifndef PATH_TRACER_PATH_TRACER_RENDERER_HPP
#define PATH_TRACER_PATH_TRACER_RENDERER_HPP

#include "renderer.hpp"

namespace pathtracer{

    // We have different types of renderers as some require different
    // setups. The default renderer shoots just one sample at the center
    // of the pixel. 
    class PathTracerRenderer : public Renderer{

    private:

        int m_samples;

    public:

        PathTracerRenderer(int width, int height, const Camera* camera, 
            const Scene* scene, const Integrator* integrator, int samples) :
            Renderer(width, height, camera, scene, integrator), 
            m_samples{samples} {}

        // The path tracer version uses multiple samples per pixel,
        // and jitters them.
        virtual Vector3 renderPixel(int i, int j) const override {

            Vector3 color(0.0);

            for(int k = 0; k < m_samples; k++){
                real offsetX = Random::next(); 
                real offsetY = Random::next();

                real x = ((i + offsetX) / m_width) * 2.0 - 1.0;
                real y = 1.0 - ((j + offsetY) / m_height) * 2.0;
                Vector2 uv(x, y);

                Ray ray = m_camera->generateRay(uv);
                color = (color * k + 
                    m_integrator->color(ray, *m_scene)) * (1.0 / (k+1.0));
            }

            return color;
        }

    };

}

#endif