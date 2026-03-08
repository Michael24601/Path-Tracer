
#ifndef PATH_TRACER_RENDERER_HPP
#define PATH_TRACER_RENDERER_HPP

#include "../camera/camera.hpp"
#include "../core/scene.hpp"
#include "../integrator/integrator.hpp"

namespace pathtracer{

    class Renderer{

    private:

        const Camera* m_camera;
        const Scene* m_scene;
        const Integrator* m_integrator;
        // Width and height in pixels
        int m_width;
        int m_height;

    public:

        Renderer(int width, int height, const Camera* camera, 
            const Scene* scene, const Integrator* integrator): 
            m_camera{camera}, m_scene{scene}, m_integrator{integrator},
            m_width{width}, m_height{height}{}

        
        std::vector<std::vector<Vector3>> render() const {

            int width = m_width;
            int height = m_height;

            std::vector<std::vector<Vector3>> c(height, 
                std::vector<Vector3>(width));

            // We map these to the center of the pixels,
            // such that the image ranged between (-1, -1) and (1, 1).
            for(int i = 0; i < width; i++){
                for(int j = 0; j < height; j++){

                    real x = ( (i + 0.5) / width ) * 2.0 - 1.0;
                    real y = ( (j + 0.5) / height ) * 2.0 - 1.0;
                    Vector2 uv(x, y);

                    Ray ray = m_camera->generateRay(uv);
                    c[j][i] = m_integrator->color(ray, *m_scene);
                }
            }

            return c;
        }

    };

}

#endif