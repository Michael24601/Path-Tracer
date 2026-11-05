
#ifndef PATH_TRACER_PIXEL_INTEGRATOR_HPP
#define PATH_TRACER_PIXEL_INTEGRATOR_HPP

#include "integrator.hpp"
#include <cassert>

namespace pathtracer{

    // Forward declarations
    class Scene;
    class Vector3;
    class Ray;

    // This is an integrator that renders a specific pixel
    class PixelIntegrator : public Integrator {

    protected:

        int m_samples;
        
    public:

        PixelIntegrator(int samples) : m_samples{samples}{
            assert(samples > 0 && "Sample count must be positive");
        }


        // Defines the rendering function as a loop over all
        // samples per pixel, then over all pixels, then shoots a ray 
        // onto each pixel from the camera.
        virtual std::vector<std::vector<Vector3>> render(
            const Scene&, const Camera&, int width, int height,
            ProgressReporter&) override;


        // Adds this virtual function that returns color given a ray
        // shot at a pixel. No default implementation. Must
        // be extended.
        virtual Vector3 color(const Ray& ray, const Scene& scene) = 0;


        // Traces one camera ray through pixel i j and and returns 
        // its color.
        Vector3 renderSample(const Scene& scene, const Camera& camera,
            int i, int j, int width, int height, bool jitter);


        // Returns true for anti aliasing
        virtual bool jitter() const = 0;
        
    };

}

#endif