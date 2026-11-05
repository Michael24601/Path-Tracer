
#ifndef PATH_TRACER_LIGHT_INTEGRATOR_HPP
#define PATH_TRACER_LIGHT_INTEGRATOR_HPP

#include "integrator.hpp"
#include <cassert>

namespace pathtracer{

    // Forward declarations
    class Scene;
    class Vector3;
    class Camera;
    class CameraConnection;

    // This is an integrator that renders a specific pixel
    class LightIntegrator : public Integrator {

    protected:

        long long m_lightPaths;

        // Splats the current position into the camera
        void splat(const Vector3& contribution,
            const CameraConnection& connection, 
            std::vector<std::vector<Vector3>>& image);

        
    public:

        LightIntegrator(long long lightPaths) : m_lightPaths{lightPaths}{
            assert(lightPaths > 0 && "Light path count must be positive");
        }


        // Defines the rendering function as a loop over all paths
        // that then get splatted into the image.
        virtual std::vector<std::vector<Vector3>> render(
            const Scene&, const Camera&, int width, int height,
            ProgressReporter&) override;


        // Adds this virtual function that traces a path and calls
        // splat to splat it onto a pixel.
        virtual void tracePath(const Scene& scene, const Camera& camera,
            std::vector<std::vector<Vector3>>& image) = 0;

        
    };

}

#endif