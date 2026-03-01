


#ifndef PATH_TRACER_SCENE_HPP
#define PATH_TRACER_SCENE_HPP

#include "instance.hpp"
#include "../intersection/rayTracer.hpp"

namespace pathtracer{

    class Scene{

    private:
    
        std::vector<Instance*> instances;


        // Also intersects a scene but quits early if one is found
        // that is closer than some distance
        Intersection intersect(const Ray& ray, real maxDistance) const {

            Intersection it = Intersection::NO_HIT;

            for(Instance* inst: instances){
                RayTracer::intersect(it, ray, inst);
                if(it.t() < maxDistance) return it;
            }

            return it;
        }



    public:

        Scene(){}


        // Expects ray this is in world coordinates.
        // Returns the closest hit.
        Intersection intersect(const Ray& ray) const {

            Intersection it = Intersection::NO_HIT;

            for(Instance* inst: instances){
                RayTracer::intersect(it, ray, inst);
            }

            return it;
        }


        bool visibility(const Vector3& origin, 
            const Vector3& target, const Vector3& direction){

            real distance = (origin - target).length();
            // We then add a spall pad to avoid self intersection
            Ray ray(origin + direction * SHADOW_EPSILON, direction);
            real maxDistance = distance - SHADOW_EPSILON;
            Intersection it = intersect(ray, maxDistance);

            // If we don't find an intersection we return true
            if(!it) return true;

            // If we find an intersection and it is closer we return
            // false as well.
            if(it.t() < maxDistance) return false;

            // Otherwise there are no obejcts in between origin and 
            // target, so we can return true.
            return true;
        }

    };

}

#endif