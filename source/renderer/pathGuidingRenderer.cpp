
#include "renderer/pathGuidingRenderer.hpp"
#include <cmath>
#include <omp.h>
#include <string>
#include "core/random.hpp"
#include "core/ray.hpp"
#include "integrator/guidable.hpp"
#include "image/imageIo.hpp"
#include "logger.hpp"
#include "math/vector2.hpp"
#include "math/vector3.hpp"
#include "pathGuiding/kdTree.hpp"
#include "pathGuiding/quadTreeUtil.hpp"
#include "core/scene.hpp"
#include "integrator/integrator.hpp"
#include "camera/camera.hpp"
#include "pathGuiding/quadTree.hpp"
#include "integrator/guidable.hpp"


namespace pathtracer{

    PathGuidingRenderer::PathGuidingRenderer(
        int width,
        int height,
        const Camera* camera,
        const Scene* scene,
        Integrator* integrator,
        int iterationCount,
        int renderSamples,
        int c, real alpha,
        int firstIterationSamples,
        int messageCount) :
        PathTracerRenderer(width, height, camera, scene, integrator, renderSamples, messageCount),
        m_firstIterationSamples{firstIterationSamples},
        m_iterationCount{iterationCount}, m_c{c} {

        // The integrator has to be guidable (contains trees)
        m_guidableIntegrator = dynamic_cast<Guidable*>(m_integrator);
        if(!m_guidableIntegrator){
            LOG_ERROR("Path guiding requires a guidable integrator");
            exit(1);
        }

        m_guidableIntegrator->setAlpha(alpha);
    }

    PathGuidingRenderer::~PathGuidingRenderer(){
        if(m_guideTree) delete m_guideTree;
        if(m_trainTree) delete m_trainTree;
    }

    void PathGuidingRenderer::preprocess(){
        
        // In the first iteration, we don't use any guide tree, only
        // the training tree. The threshold is c * 2^k where k = 0.

        int threshold = m_c;

        // If guiding tree is nullptr, then they just use bsdf.
        // If training tree is nullptr, then they just render and don't train.
        // If neither is nullptr, then they train using guiding tree.
        m_trainTree = new KdTree(threshold);
        m_guideTree = nullptr;

        KdTree* tree;

        // Saves the number of samples needed for the actual final render
        // as it will be overwritten during training.
        int renderSamples = m_samples;

        // Samples used
        m_samples = m_firstIterationSamples;
        // For logging, we set up the number of paths in total

        m_totalPathCount = static_cast<long long>(m_width) * m_height * m_samples;
        m_pathCounter = 0;

        // Then loops
        for(int k = 0; k < m_iterationCount; k++){

            // We set both
            m_guidableIntegrator->setGuideTree(m_guideTree);
            m_guidableIntegrator->setTrainTree(m_trainTree);

            // Does one iteration
            renderAllPixels();

            // We recompute the trees and swap them
            m_trainTree->recomputeSampleWeight();

            if(m_guideTree) {
                delete m_guideTree;
            }

            m_guideTree = m_trainTree->copyTree();

            // Training tree is reset
            // Threshold is recomputed
            threshold = m_c * sqrtReal(static_cast<real>(std::pow(2, k)))
                * (m_firstIterationSamples / 4);
            m_trainTree->setThreshold(threshold);
            m_trainTree->adaptTree();
            m_trainTree->reset();

            // We update samples, counter, and total path count
            m_samples *= 2;
            m_totalPathCount = static_cast<long long>(m_width) * m_height * m_samples;
            m_pathCounter = 0;

            LOG_INFO("Iteration: " + std::to_string(k) + " was completed.");
        }

        // After this, render be called again, for the last time,
        // and actually saved. So we need to set up the trees to be correct.
        
        // The guide tree is the training tree from before.
        m_guidableIntegrator->setGuideTree(m_guideTree);
        // The train tree is null
        m_guidableIntegrator->setTrainTree(nullptr);

        // The number of samples is the one initially reserved
        // for rendering (the one we preserved).
        m_samples = renderSamples;
        // We can also update total path count for logging
        m_totalPathCount = static_cast<long long>(m_width) * m_height * m_samples;
    }


    std::string PathGuidingRenderer::toString() const {
        return m_integrator->toString() + " SPP=" 
            + std::to_string(m_samples) + " ITERATIONS=" 
            + std::to_string(m_iterationCount) + " "
            + std::to_string(m_height) + "x" 
            + std::to_string(m_width) + "px";
    }

}