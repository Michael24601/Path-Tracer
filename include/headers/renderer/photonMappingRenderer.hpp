
#ifndef PATH_TRACER_PHOTON_MAPPING_RENDERER_HPP
#define PATH_TRACER_PHOTON_MAPPING_RENDERER_HPP

#include "renderer.hpp"
#include "photonMapping/photonMap.hpp"

namespace pathtracer{

    // We have different types of renderers as some require different
    // setups.
    class PhotonMappingRenderer : public Renderer{

    protected:

        int m_photonCount;
        int m_maxDepth;
        int m_threshold;

        PhotonMap* m_map;

    public:

        PhotonMappingRenderer(int width, int height, const Camera* camera,
            const Scene* scene, Integrator* integrator, int photonCount,
            int threshold, int maxDepth, int messageCount);

        
        ~PhotonMappingRenderer();


        // Traces the photons and builds a photon map
        virtual void preprocess() override;

    };

}

#endif