
#ifndef PATH_TRACER_RENDERER_HPP
#define PATH_TRACER_RENDERER_HPP

#include <vector>
#include "config.hpp"
#include <atomic>
#include <string>

namespace pathtracer{

    class Camera;
    class Scene;
    class Integrator;
    class Vector2;
    class Vector3;

    class Renderer{

    protected:

        const Camera* m_camera;
        
        // Width and height in pixels
        int m_width;
        int m_height;

        // The total number of paths to shoot, a counter, and a rate 
        // for logging.
        long long m_totalPathCount;
        std::atomic<long long> m_pathCounter;
        int m_messageCount;


    public:

        const Scene* m_scene;

        Integrator* m_integrator;

        Renderer(int width, int height,
            const Camera* camera, const Scene* scene,
            Integrator* integrator, int messageCount = 10);


        // This is called in render before rendering, can be used to train
        // or set up the integrator like for guiding.
        // It also has to set things like the totalPathCount.
        virtual void preprocess();


        // Calls preprocess then renders all pixels
        std::vector<std::vector<Vector3>> render();
        

        // This shoots rays for every pixel
        std::vector<std::vector<Vector3>> renderAllPixels();


        // This function actually calls renderPixel, and sets the
        // appropriate offset, possibly loops if there is a large
        // sample count. It also decides on the offset.
        virtual Vector3 renderPixel(int i, int j);


        // This just shoots a ray, using the supplied offsets for pixels,
        // and returns rendered colors. It also updates sample counter.
        Vector3 shootRay(int i, int j, real offsetX, real offsetY);


        virtual std::string toString() const;

    };

}

#endif