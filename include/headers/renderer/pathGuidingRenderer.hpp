
#ifndef PATH_TRACER_PATH_GUIDING_RENDERER_HPP
#define PATH_TRACER_PATH_GUIDING_RENDERER_HPP

#include "renderer.hpp"
#include <vector>

namespace pathtracer{

    class KdTree;

    // We have different types of renderers as some require different
    // setups. The default renderer shoots just one sample at the center
    // of the pixel. 
    class PathGuidingRenderer : public Renderer{

    private:

        int m_renderSamples;
        // Samples used to train first iteration (doubled each iteration)
        int m_firstIterationSamples;
        // Number of iterations 
        int m_iterationCount;

        // Actual samples used to render
        int m_samplesUsed;

        // The c used to determine when to subdivide spatial tree
        int m_c;

        // The SD trees used for path guiding.
        // The first guides the second (ieration k and k+1)
        KdTree* m_guideTree;
        KdTree* m_trainTree;

    public:

        PathGuidingRenderer(int width, int height, const Camera* camera,
            const Scene* scene, Integrator* integrator,
            int firstIterationSamples, int iterationCount,
            int renderSamples, int c);

        ~PathGuidingRenderer();

        std::vector<std::vector<Vector3>> render() override;

        // The path tracer version uses multiple samples per pixel,
        // and jitters them.
        Vector3 renderPixel(int i, int j) override;
    };

}

#endif