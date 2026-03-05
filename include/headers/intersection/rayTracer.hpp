
#ifndef PATH_TRACER_RAY_TRACER_HPP
#define PATH_TRACER_RAY_TRACER_HPP

#include "../core/instance.hpp"
#include "intersectionUtil.hpp"
#include "IntersectionList.hpp"

namespace pathtracer{

    class RayTracer{

    private:

        // Transforms intersections from local to world coordinates
        static void transformIntersection(Intersection& it, 
            const Instance* const instance, const Ray& globalRay){
            
            it.setPosition(instance->transform().transform(it.position()));
            it.setTangent(
                instance->transform().transformDirection(it.tangent()));
            it.setGeometryNormal(
                instance->transform().transformNormal(it.geometryNormal()));
            it.setShadingNormal(
                instance->transform().transformNormal(it.shadingNormal()));
            it.setT((it.position() - globalRay.origin()).length());
            it.setInstance(instance);
        }

    public:

        // Intersects a ray with a shape instance. If we have a
        // closer intersection, we use it.
        static void intersect(Intersection& oldIt, const Ray& ray,
            const Instance* const instance){

            // First we transform the ray to local coordinates
            Transform inv = instance->transform().inverse();
            Ray localRay = inv.transform(ray);

            // Then we intersect the shape
            IntersectionList list; 
            instance->shape()->intersect(localRay, list);

            if(list.empty()) return;

            // The intersection distance t of the old intersection
            // is scaled so that we can compare it to the new
            // intersection.
            Vector3 oldLocalIntersectionPoint = 
                inv.transform(oldIt.position());
            real oldLocalT = 
                (oldLocalIntersectionPoint - localRay.origin()).length();

            while(!list.empty()){

                // Since the list is a min heap with respect to the
                // distance t, we can disregard the whole list if
                // the top element is further than old distance t.
                if(list.top()->t() >= oldLocalT){
                    break;
                }

                // We discard the hit if it is negative or self intersecting
                if(IntersectionGuards::selfIntersection(*list.top())){
                    list.pop();
                    continue;
                }

                // We then do some alpha masking
                bool hit = AlphaMasking::alphaMask(*list.top(), instance);
                // If not, it means we passed through
                if(!hit){
                    list.pop();
                    continue;
                }

                // Finally, if we get to this point, we know that
                // we have a new closest intersection, so we can 
                // transform it to world coordinates, and replace the
                // older one.
                oldIt = *list.top();
                transformIntersection(oldIt, instance, ray);
                // We also compute the shading frame now
                oldIt.computeShadingFrame();
                break;
            }
        }

    };

}

#endif