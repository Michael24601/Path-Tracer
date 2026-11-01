

#ifndef PATH_TRACER_INTEGRATOR_HPP
#define PATH_TRACER_INTEGRATOR_HPP

#include <string>
#include <atomic>

namespace pathtracer{

    // Forward declarations
    class Scene;
    class Ray;
    class Vector3;

    class Integrator{

    public:

        // Used for debugging
        std::atomic<long long> bounceCount{0};
    
        // Uses whatever rendering technique it wants to render
        // the light received by the given shadow ray.
        virtual Vector3 color(const Ray&, const Scene&) = 0;


        virtual std::string toString() const = 0;


        virtual ~Integrator() = default;


        // This is empty unless overriden
        virtual void printStatistics() const {}
     
    };

}

#endif