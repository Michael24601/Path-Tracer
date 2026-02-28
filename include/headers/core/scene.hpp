


#ifndef PATH_TRACER_SCENE_HPP
#define PATH_TRACER_SCENE_HPP

#include "instance.hpp"
#include "../intersection/rayTracer.hpp"

namespace pathtracer{

    class Scene{

    private:
    
        std::vector<Instance*> instances;

    public:

        Scene(){}


        // Expects ray this is in world coordinates
        Intersection intersect(const Ray& ray) const {

            Intersection it = Intersection::NO_HIT;

            for(Instance* inst: instances){
                RayTracer::intersect(it, ray, inst);
            }

            return it;
        }
               
    };

}

#endif