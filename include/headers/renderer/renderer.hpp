
#ifndef PATH_TRACER_RENDERER_HPP
#define PATH_TRACER_RENDERER_HPP

#include <vector>
#include <string>
#include "progressReporter.hpp"

namespace pathtracer{

    class Camera;
    class Scene;
    class Integrator;
    class Vector3;

    class Renderer{

    protected:

        // Not owned by the renderer
        const Camera* m_camera;
        const Scene* m_scene;

        ProgressReporter reporter;

        // Width and height in pixels
        int m_width;
        int m_height;

        // Renderer owns integrator, and has to delete it
        Integrator* m_integrator;

    public:


        Renderer(const Scene* scene, const Camera* camera, 
            Integrator* integrator, int width, 
            int height, int messageCount = 10) : m_scene{scene},
            m_camera{camera}, m_integrator{integrator}, 
            m_height{height}, m_width{width}, reporter(messageCount){}


        ~Renderer();


        // Renders the scene by calling render in integrator
        std::vector<std::vector<Vector3>> render();


        std::string toString() const;


        void printStatistics() const;
    };

}

#endif