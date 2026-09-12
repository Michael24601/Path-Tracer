
#ifndef PATH_TRACER_RAY_TRACER_HPP
#define PATH_TRACER_RAY_TRACER_HPP

#include "../core/instance.hpp"
#include "../logger.hpp"
#include "intersection.hpp"

namespace pathtracer{

    class RayTracer{

    private:

        // Assuming we have a valid intersection, the alpha masking
        // function probabilistically chooses to either go through
        // or intersect the surface at this point, given the alpha
        // value at the intersection and using it as a probability.
        // Note that we input instance alongside the intersection
        // instead of using the instance parameter since it may not
        // have been set yet.

        // Note intersection is assumed to still be in local coordinates.
        // So no need to transform it for now.
        // Note that both localRay and intersection are changed.
        static bool alphaMask(Intersection& it, Ray& localRay, real oldT,
            const Instance* instance){

            // First we extract the alpha value if available
            // If not, then we record the hit as having worked.
            if(!instance->hasAlphaTexture()){
                return true;
            }

            bool pass = true;

            real currentOldT = oldT;
            // Difference keeps track of the offset applied
            real difference{0.0};
            
            while(it && it.t() < currentOldT && it.t() > SHADOW_EPSILON && pass){
                
                real alpha = instance->alphaTexture()->sample(it.uv()).x();
                real random = Random::next();
                // If alpha is 0, it means that we will always pass through.
                // If 1, it means we never pass through.
                pass = random > alpha;

                if(!pass){ break; }

                // We update oldT since origin of ray is changing
                difference = difference + (localRay.origin() - it.position()).length() 
                    + SHADOW_EPSILON;
                
                // If oldT is infinite, no need to even bother offsetting it
                if(oldT < REAL_INFINITY){
                    currentOldT = oldT - difference;
                }

                // Then we reintersect the same shape if we pass
                // We define a new (local) ray to do that.
                localRay.setOrigin(it.position() + localRay.direction() * SHADOW_EPSILON);

                it = instance->shape()->intersect(localRay, currentOldT);
            }


            // When we are out, if we have a valid intersection, we
            // update t to reflect the original origin of the ray.
            if(!pass && it && it.t() < currentOldT && it.t() > SHADOW_EPSILON){
                it.setT(it.t() + difference);
                return true;
            }
                
            return false;
        
        }

        
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
        static bool intersect(Intersection& oldIt, const Ray& ray,
            const Instance* const instance){

            // First we transform the ray to local coordinates
            // (But we don't normalize direction yet since we need
            // the scaling factor)
            Transform inv = instance->transform().inverse();
            Ray localRay = inv.transformKeepScale(ray);

            // Then we transform the intersection t (distance)
            // into local coordinates using the scale.
            // Scale is the transformed direction length assuming that
            // the direction was actually normalized to begin with.
            real scale = localRay.direction().length();
            
            real oldT;
            if(oldIt){ oldT = oldIt.t() * scale; }
            // If the oldIt is not a valid hit, then oldT is the max value
            else{ oldT = REAL_INFINITY; }
            
            // Now we can normalize the ray direction
            localRay.setDirection(localRay.direction() / scale);

            // Then we intersect the shape
            Intersection it = instance->shape()->intersect(localRay, oldT);

            // If we have an intersection and it actually is closer
            // (could or could not have been checked inside).
            if(it && it.t() > SHADOW_EPSILON && it.t() < oldT){

                // Before we process the intersection, we do
                // alpha masking.
                bool hit = alphaMask(it, localRay, oldT, instance);

                if(hit){
                    // Finally, if we get to this point, we know that
                    // we have a new closest intersection, so we can 
                    // transform it to world coordinates, and replace the
                    // older one.
                    it.setInstance(instance);
                    SurfacePoint newIt = instance->transform().transformSurfacePoint(it);
                    real newT = it.t() / scale;
                    oldIt = Intersection(newT, newIt);
                    // We also compute the shading frame now
                    oldIt.computeShadingFrame();

                    // Since we found the closest hit, we can now do normal mapping
                    normalMapping(oldIt, instance);

                    return true;
                }

            }

            return false;
        }

    };

}

#endif