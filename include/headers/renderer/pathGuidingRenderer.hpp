
#ifndef PATH_TRACER_PATH_GUIDING_RENDERER_HPP
#define PATH_TRACER_PATH_GUIDING_RENDERER_HPP

#include "renderer.hpp"
#include "../pathGuiding/kdTree.hpp"
#include "../integrator/guidable.hpp"
#include "../pathGuiding/quadTreeUtil.hpp"
#include "../imageIo/imageIo.hpp"

namespace pathtracer{

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
            int firstIterationSamples, int iterationCount, int renderSamples,
            int c) : Renderer(width, height, camera, scene, integrator), 
            m_firstIterationSamples{firstIterationSamples},
            m_iterationCount{iterationCount},
            m_renderSamples{renderSamples}, m_c{c} {}

        
        ~PathGuidingRenderer(){
            if(m_guideTree) delete m_guideTree;
            if(m_trainTree) delete m_trainTree;
        }


        virtual std::vector<std::vector<Vector3>> render() {

            int width = m_width;
            int height = m_height;

            //---------------------- PATH GUIDING LOGIC ------------------------
            
            // In the first iteration, we don't use any guide tree, only
            // the training tree. The threshold is c * 2^k where k = 0.

            int threshold = m_c;

            // If guiding tree is nullptr, then they just use bsdf.
            // If training tree is nullptr, then they just render and don't train.
            // If neither is nullptr, then they train using guiding tree.
            m_trainTree = new KdTree(threshold);
            m_guideTree = nullptr;

            // Samples used
            m_samplesUsed = m_firstIterationSamples;

            // The integrator has to be guidable (contains trees)
            Guidable* integrator = dynamic_cast<Guidable*>(m_integrator);
            if(!integrator){
                LOG_ERROR("Path guiding requires a guidable integrator");
            }

            // Then loops
            for(int k = 0; k < m_iterationCount; k++){

                // We set both
                integrator->setGuideTree(m_guideTree);
                integrator->setTrainTree(m_trainTree);

                // Does one iteration
                #pragma omp parallel for
                for(int j = 0; j < height; j++){
                    for(int i = 0; i < width; i++){
                        renderPixel(i, j);
                    }
                }

                // We recompute the trees and swap them
                m_trainTree->recomputeSampleCount();

                if(m_guideTree) {
                    delete m_guideTree;
                }
                m_guideTree = m_trainTree->copyTree();

                // Training tree is reset
                m_trainTree->adaptTree();
                m_trainTree->reset();

                threshold = m_c * sqrt(static_cast<float>(std::pow(2, k)));
                m_samplesUsed *= 2;

                m_trainTree->setThreshold(threshold);
                  
                Vector3 p(1, 0.53, 1.49);
                p = p - m_scene->getBoundingBox().minCorner();
                p = p / (m_scene->getBoundingBox().maxCorner() - m_scene->getBoundingBox().minCorner());
                auto tree = m_guideTree->getDTree(p);
                auto im = QuadTreeUtil::renderQuadTree(tree,  512);
                ImageIo::savePNG(im, "output/file" + std::to_string(k) + ".png");
                

                LOG_INFO("Iteration: " + std::to_string(k));
            }

            //--------------------------- RENDERING ----------------------------

            std::vector<std::vector<Vector3>> color(height, 
                std::vector<Vector3>(width));

            integrator->setGuideTree(m_guideTree);
            integrator->setTrainTree(nullptr);

            m_samplesUsed = m_renderSamples;

            #pragma omp parallel for
            for(int j = 0; j < height; j++){
                for(int i = 0; i < width; i++){
                    color[j][i] = renderPixel(i, j) ;
                }
            }

            return color;
        }

        

        // The path tracer version uses multiple samples per pixel,
        // and jitters them.
        virtual Vector3 renderPixel(int i, int j) override {

            Vector3 color(0.0);

            for(int k = 0; k < m_samplesUsed; k++){
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