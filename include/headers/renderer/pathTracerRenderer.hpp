
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
            const Scene* scene, Integrator* integrator, int samples);

        // The path tracer version uses multiple samples per pixel,
        // and jitters them.
        Vector3 renderPixel(int i, int j) override;
    };

}

#endif