
#include "renderer/photonMappingRenderer.hpp"
#include "light/lightSample.hpp"
#include "math/mathUtil.hpp"
#include "core/sceneUtil.hpp"
#include "light/light.hpp"
#include "photonMapping/photonMap.hpp"
#include "core/random.hpp"
#include "core/ray.hpp"
#include "math/vector2.hpp"
#include "math/vector3.hpp"
#include "camera/camera.hpp"
#include "integrator/integrator.hpp"
#include "core/scene.hpp"
#include "integrator/photonMappingIntegrator.hpp"
#include "photonMapping/photonMappingUtil.hpp"
#include "logger.hpp"


namespace pathtracer{


    PhotonMappingRenderer::PhotonMappingRenderer(int width, int height,
        const Camera* camera, const Scene* scene, Integrator* integrator,
        int photonCount, int threshold, int maxDepth, int messageCount) :
        Renderer(width, height, camera, scene, integrator, messageCount),
        m_photonCount{photonCount}, m_maxDepth{maxDepth}, 
        m_threshold{threshold}, m_map{nullptr} {}


    PhotonMappingRenderer::~PhotonMappingRenderer(){
        if(m_map){
            delete m_map;
        }
    }


    void PhotonMappingRenderer::preprocess(){

        // Total number of messages to print
        m_totalPathCount = static_cast<long long>(m_width) * m_height;

        // Once we have our photons, we can create a map
        m_map = PhotonMapping::tracePhotons(m_photonCount, m_maxDepth, 
            m_threshold, *m_scene);

        PhotonMappingIntegrator* pmi = 
            dynamic_cast<PhotonMappingIntegrator*>(m_integrator);
        if(!pmi){
            LOG_ERROR("Wrong integrator type.");
            exit(1);
        }

        pmi->setPhotonMap(m_map);
        pmi->setPhotonCount(m_photonCount);
    }


}