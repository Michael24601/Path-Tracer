
#ifndef PATH_TRACER_RAY_TRACER_HPP
#define PATH_TRACER_RAY_TRACER_HPP

#include "../core/instance.hpp"
#include "intersectionUtil.hpp"
#include "IntersectionList.hpp"

namespace pathtracer{

    class RayTracer{


    private:

        
        static void normalMapping(Intersection& it, const Instance* const inst){

            if(inst->normalTexture()){

                Vector3 normal = inst->normalTexture()->sample(it.uv());
                // From [0,1] to [-1,1]
                normal = normal * 2.0f - Vector3(1.0f);

                // We need to map normal from shading frame to world coordinates
                // We can't use normal transform since it applies translation
                // (not suitable for directions).
                // We should in theory use transformNormal which was designed
                // for normals, but since shadingFrame is always orthogonal,
                // transformDirection is equivalent, while much cheaper to
                // compute, so we will use that.
                // We can also use transformNormal, but it is more expensive
                // and adds nothing in this specific case.
                normal = it.shadingFrame().transformDirection(normal);
                // Then the frame is updated
                it.setShadingNormal(normal);
                // Frame is recomputed
                it.computeShadingFrame();
            }
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
            // This is a priority queue so top is always the closest
            // current hit.
            IntersectionList list; 
            instance->shape()->intersect(localRay, list);

            if(list.empty()) {
                return;
            }

            // The intersection distance t of the old intersection
            // is scaled so that we can compare it to the new
            // intersection.
            real oldLocalT;

            // If it's not a no hit
            if(oldIt){
                Vector3 oldLocalIntersectionPoint = 
                    inv.transform(oldIt.position());
                oldLocalT = (oldLocalIntersectionPoint 
                    - localRay.origin()).length();
            }
            else{
                oldLocalT = REAL_INFINITY;
            }

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
                oldIt.setInstance(instance);
                SurfacePoint newIt = instance->transform()
                    .transformSurfacePoint(oldIt);
                real newT = (newIt.position() - ray.origin()).length();
                oldIt = Intersection(newT, newIt);
                // We also compute the shading frame now
                oldIt.computeShadingFrame();

                // Since we found the closest hit, we can now do normal mapping
                normalMapping(oldIt, instance);

                break;
            }
        }

    };

}

#endif