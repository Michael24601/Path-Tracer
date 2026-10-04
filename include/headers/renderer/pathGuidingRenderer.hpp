
#ifndef PATH_TRACER_PATH_GUIDING_RENDERER_HPP
#define PATH_TRACER_PATH_GUIDING_RENDERER_HPP

#include "pathTracerRenderer.hpp"
#include <vector>

namespace pathtracer{

    class KdTree;

    // We have different types of renderers as some require different
    // setups. The default renderer shoots just one sample at the center
    // of the pixel. 
    class PathGuidingRenderer : public PathTracerRenderer{

    private:

        // Samples used to train first iteration (doubled each iteration)
        int m_firstIterationSamples;
        // Number of iterations 
        int m_iterationCount;

        // Actual samples used to render
        int m_samplesUsed;

        // The c used to determine when to subdivide spatial tree
        int m_c;

        // The SD trees used for path guiding.
        // The first guides the second trains (ieration k+1 and k)
        KdTree* m_guideTree;
        KdTree* m_trainTree;

    public:

        PathGuidingRenderer(int width, int height, const Camera* camera,
            const Scene* scene, Integrator* integrator,
            int firstIterationSamples, int iterationCount,
            int renderSamples, int c, int messageCount = 10);

        ~PathGuidingRenderer();

        // Trains tree here by calling renderAllPixels
        void preprocess() override;


        // Uses the same renderPixel() as pathtracer
    };

}

#endif