
#include "integrator/pathTracerSmsGuided.hpp"
#include "bsdf/bsdfSample.hpp"
#include "core/instance.hpp"
#include "core/random.hpp"
#include "core/ray.hpp"
#include "core/scene.hpp"
#include "intersection/intersection.hpp"
#include "math/mathUtil.hpp"
#include "pathGuiding/kdTree.hpp"
#include "pathGuiding/quadTree.hpp"
#include "math/constants.hpp"
#include "logger.hpp"
#include "bsdf/bsdf.hpp"
#include "sms/specularManifoldSampling.hpp"
#include "logger.hpp"

namespace pathtracer{


    PathTracerSmsGuided::PathTracerSmsGuided(int samples, int maxDepth, 
        bool useHalfVector, bool biased, bool twoStage, int newtonIterations, 
        int unbiasedTrials, int biasedTrials, real alpha, int initialSamples, 
        int iterations, real c,  const Scene& scene) : 
        PixelIntegratorGuided(samples, alpha, initialSamples,
            iterations, c), m_maxDepth{maxDepth}, sms{nullptr}{

        assert(maxDepth > 0 && "Max depth must be positive");    
        
        sms = new SpecularManifoldSampling(scene, useHalfVector, biased, 
            twoStage, newtonIterations, unbiasedTrials, biasedTrials);
    }

    
    PathTracerSmsGuided::~PathTracerSmsGuided(){
        if(sms){
            delete sms;
        }
    }


    Vector3 PathTracerSmsGuided::color(const Ray& ray, const Scene& scene) {

        Ray currRay = ray;
        Vector3 color = Vector3(0.0);
        Vector3 throughput = Vector3(1.0);

        // Alternate throughput for RR that uses bsdf pdf
        Vector3 rrWeight = Vector3(1.0);

        // Used for path guiding training
        std::vector<SurfacePoint> positions;
        std::vector<Vector3> weights;
        std::vector<Vector3> directions;
        std::vector<real> pdfs;
        std::vector<real> cosines;
        std::vector<Vector3> contributions;
        Vector3 finalEmission(0.0);

        // saves the last sms sample bounce index
        const Instance* lastInst = nullptr;

        for(int i = 0; i < m_maxDepth; i++){

            Vector3 wo = -currRay.direction();

            // First we intersect the scene
            m_bounceCount++;
            Intersection it = scene.intersect(currRay);
            // If no hits (we can break or sample envmap)
            if(!it){
                break;
            }

            
            // If we have an emissive surface, add emission and break.
            // If last hit was a specular (can cast caustic), we ignore it,
            // since it would have double counted SMS sample.
            // If last hit not specular, we can also add its contribution,
            // as no double counting.
            if(it.instance()->emission()){
                if (i == 0 || i == 1 || !lastInst->bsdf()->isSpecular()){

                    Vector3 emission = it.evaluateEmission(wo);

                    color = color + emission * throughput;

                    finalEmission = emission;
                }
                break;
            }

            lastInst = it.instance();


            //---------------------------- SMS -----------------------------

            Vector3 contribution(0.0);

            // If the current surface is neither an emission nor specular,
            // we can try doing SMS.
            // We don't use guiding here, but we add the contributions
            // for the training down the line (NOT at this vertex).
            if(!it.instance()->bsdf()->isSpecular() && !it.instance()->emission() &&
                it.instance()->isCausticReceiver()){

                std::vector<SmsOutput> outputs = sms->sample(it, wo, &scene);

                // If it worked, we can just return contribution
                // weighted by throughput. Otherwise it is 0.0 anyway.
                Vector3 contribution(0.0);
                
                for(const SmsOutput& output: outputs){

                    BsdfSample eval = it.evaluateBsdf(wo, output.wi);
                    Vector3 c = output.contribution * eval.bsdf() * eval.cosine();

                    contribution = contribution + c;
                }

                color = color + contribution * throughput;
            }


            // We add it here, be it 0.0 or whatever SMS contributed
            contributions.push_back(contribution);


            // ------ This next step is normal pathtracer ------

            // Russian roulette
            real p = Util::russianRoulette(rrWeight, i);
            if (Random::next() > p){
                break;
            }

            // If we have a non-null guide tree, we can use it to guide
            // samples.

            // ---------------------- PATH GUIDING ------------------------

            // This is sampled using the guided tree (if it exists),
            // with fallbacks to bsdf handled already.
            GuidedSample sample = sampleGuided(wo, it, scene);

            if(!sample.isValid){
                break;
            }

            if(sample.isGuided){
                // We need to compute the bsdf weight for the sake of RR
                Vector3 weight = it.evaluateBsdf(wo, sample.wi).weight();
                rrWeight = rrWeight * weight * (1.0f / p);
            }
            else{
                // If it is not guided, we just use it as is
                rrWeight = rrWeight * sample.weight * (1.0f / p);
            }

            // And we update the throughput (along with RR probability)
            sample.weight = sample.weight * (1.0f / p);

            // We then store, in a stack, the position, and the throughput
            // at this point.
            if(m_trainTree){
                weights.push_back(sample.weight);
                positions.push_back(it);
                directions.push_back(sample.wi);
                pdfs.push_back(sample.pdf);
                cosines.push_back(sample.cosine);
            }

            throughput = throughput * sample.weight;
            // The new ray starts at the last intersected point
            // and points towards the new intersected point.
            currRay = Ray(it.position() + sample.wi * SHADOW_EPSILON, sample.wi);

        }


        // Saved for training
        recordPath(positions, directions, weights, pdfs, cosines, 
            contributions, finalEmission, scene);

        return color;
    }


    void PathTracerSmsGuided::printStatistics(int width, int height) const {

        long long total = static_cast<long long>(width) * height * m_samples;
        real averageBounceCount = static_cast<real>(m_bounceCount) / total;

        LOG_DEBUG("Bounce count: " + std::to_string(m_bounceCount));
        LOG_DEBUG("Average bounce count: " + std::to_string(averageBounceCount));

        m_guideTree->printStatistics();
        sms->printStatistics();
    }


    std::string PathTracerSmsGuided::toString() const{
        return "PT " + sms->toString() 
            + " GUIDED ALPHA=" + std::to_string(m_alpha) 
            + " TRAINING_IT=" + std::to_string(m_iterations) 
            + " SPP=" + std::to_string(m_samples);
    }


}