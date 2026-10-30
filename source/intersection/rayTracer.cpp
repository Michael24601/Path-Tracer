
#include "intersection/rayTracer.hpp"
#include "core/instance.hpp"
#include "core/random.hpp"
#include "core/ray.hpp"
#include "intersection/hit.hpp"
#include "intersection/surfacePoint.hpp"
#include "shapes/shape.hpp"
#include "texture/texture.hpp"
#include "math/constants.hpp"
#include "logger.hpp"

namespace pathtracer{

    namespace RayTracer{

        bool alphaMask(Hit& hit, Ray& localRay, real oldT,
            const Instance* instance) {

            // First we extract the alpha value if available
            // If not, then we record the hit as having worked.
            if(!instance->hasAlphaTexture()){
                return true;
            }

            bool pass = true;
            bool success = true;

            // Difference keeps track of the offset applied
            real difference{0.0};

            while(success && hit.t > SHADOW_EPSILON && pass) {

                Vector2 uv = instance->shape()->getUv(hit, localRay);
                real alpha = instance->alphaTexture()->sample(uv).x();

                real random = Random::next();

                // If alpha is 0, it means that we will always pass through.
                // If 1, it means that we never pass through.
                pass = random > alpha;

                if(!pass){
                    // We got a hit here, so we just return, but first
                    // we adjust to reflect the original ray origin.
                    hit.t = hit.t + difference;
                    return true;
                }

                // We update oldT since origin of ray is changing
                real offset = hit.t + SHADOW_EPSILON;
                difference += offset;

                // New upperbound, distance until the oldHit we had
                if(oldT < REAL_INFINITY) {
                    hit.t = oldT - difference;

                    // This means we already exceeded it
                    if(hit.t <= SHADOW_EPSILON) {
                        return false;
                    }
                }
                // If the old hit is infinitely far away, we can continue 
                else {
                    hit.t = REAL_INFINITY;
                }

                // Then we reintersect the same shape if we pass
                // We define a new (local) ray to do that.
                localRay.setOrigin(localRay.origin()
                    + localRay.direction() * offset);

                // This won't return a success if the new hit as a smaller
                // t than hit. It also won't overwrite hit.t if not
                // a success.
                success = instance->shape()->intersect(localRay, hit);
            }

            return false;
        }
        

        // Intersects a ray with a shape instance. If we have a
        // closer intersection, we use it.
        bool intersect(Hit& oldHit, const Ray& ray, 
            const Instance* const instance) {

            // First we transform the ray to local coordinates
            // (But we don't normalize direction yet since we need
            // the scaling factor)

            Ray localRay = instance->inverse().transformKeepScale(ray);

            // Then we transform the intersection t (distance)
            // into local coordinates using the scale.
            // Scale is the transformed direction length assuming that
            // the direction was actually normalized to begin with.
            real scale = localRay.direction().length();

            Hit localHit;
            // Old t is the old T scaled in local coordinates
            real oldT;
            if(oldHit.t < REAL_INFINITY){
                oldT = oldHit.t * scale;
            }
            else{
                oldT = REAL_INFINITY;
            }
            localHit.t = oldT;

            // Now we can normalize the ray direction
            localRay.setDirection(localRay.direction() / scale);

            // Then we intersect the shape
            bool success = instance->shape()->intersect(localRay, localHit);

            // If we have an intersection and it actually is closer
            // (could or could not have been checked inside).
            if(success && localHit.t > SHADOW_EPSILON && localHit.t < oldT) {

                // Before we process the intersection, we do
                // alpha masking.
                bool hit = alphaMask(localHit, localRay, oldT, instance);

                if(hit){

                    // Finally, if we get to this point, we know that
                    // we have a new closest intersection, so we can
                    // transform it to world coordinates.
                    localHit.instance = instance;
                    localHit.t = localHit.t / scale;

                    oldHit = localHit;

                    return true;
                }
            }   

            return false;
        }

    }

}