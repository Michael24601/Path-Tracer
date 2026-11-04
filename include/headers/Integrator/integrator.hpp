

#ifndef PATH_TRACER_INTEGRATOR_HPP
#define PATH_TRACER_INTEGRATOR_HPP

#include <string>
#include <vector>

namespace pathtracer{

    // Forward declarations
    class Scene;
    class Vector3;
    class Camera;
    class ProgressReporter;

    class Integrator {
        
    public:

    
        Integrator(){}


        virtual ~Integrator() = default;


        virtual std::vector<std::vector<Vector3>> render(
            const Scene&, const Camera&, int width, int height,
            ProgressReporter&) = 0;


        virtual void printStatistics(int width, int height) const = 0;


        virtual std::string toString() const = 0;
        
    };

}

#endif