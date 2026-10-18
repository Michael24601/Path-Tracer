
#ifndef PATH_TRACER_PATH_GUIDING_RENDERER_HPP
#define PATH_TRACER_PATH_GUIDING_RENDERER_HPP

#include "pathTracerRenderer.hpp"
#include "integrator/guidable.hpp"
#include <vector>

namespace pathtracer{

    class KdTree;

    // We have different types of renderers as some require different
    // setups. 
    class PathGuidingRenderer : public PathTracerRenderer{

    private:

        // Samples used to train first iteration (doubled each iteration)
        int m_firstIterationSamples;
        // Number of iterations 
        int m_iterationCount;

        // The c used to determine when to subdivide spatial tree
        int m_c;

        // The SD trees used for path guiding.
        // The first guides the second trains (ieration k+1 and k)
        KdTree* m_guideTree;
        KdTree* m_trainTree;

        // Stores downcast integrator as well
        Guidable* m_guidableIntegrator;

    public:

        PathGuidingRenderer(int width, int height, const Camera* camera,
            const Scene* scene, Integrator* integrator, int iterationCount,
            int renderSamples, int c, real alpha, int firstIterationSamples = 4,
            int messageCount = 10);

        ~PathGuidingRenderer();

        // Trains tree here by calling renderAllPixels
        void preprocess() override;


        // Uses the same renderPixel() as pathtracer


        std::string toString() const override;
    };

}

#endif