
#ifndef PATH_TRACER_RENDERER_HPP
#define PATH_TRACER_RENDERER_HPP

#include <vector>
#include "config.hpp"

namespace pathtracer{

    class Camera;
    class Scene;
    class Integrator;
    class Vector2;
    class Vector3;

    class Renderer{

    protected:

        const Camera* m_camera;
        const Scene* m_scene;
        // Width and height in pixels
        int m_width;
        int m_height;

    public:

        Integrator* m_integrator;

        Renderer(
            int width,
            int height,
            const Camera* camera,
            const Scene* scene,
            Integrator* integrator);

        virtual std::vector<std::vector<Vector3>> render();

        // Default version just calls the integrator at midpoint of pixel
        virtual Vector3 renderPixel(int i, int j);
    };

}

#endif