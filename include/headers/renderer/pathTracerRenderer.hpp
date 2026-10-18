
#ifndef PATH_TRACER_PATH_TRACER_RENDERER_HPP
#define PATH_TRACER_PATH_TRACER_RENDERER_HPP

#include "renderer.hpp"

namespace pathtracer{

    // We have different types of renderers as some require different
    // setups.
    class PathTracerRenderer : public Renderer{

    protected:

        int m_samples;

    public:

        PathTracerRenderer(int width, int height, const Camera* camera,
            const Scene* scene, Integrator* integrator, int samples, 
            int messageCount = 10);

        virtual void preprocess() override;

        // The path tracer version uses multiple samples per pixel,
        // and jitters them.
        virtual Vector3 renderPixel(int i, int j) override;


        std::string toString() const override;
    };

}

#endif