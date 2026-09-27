#include "core/scene.hpp"
#include <cassert>
#include "core/instance.hpp"
#include "core/ray.hpp"
#include "intersection/rayTracer.hpp"
#include "intersection/intersection.hpp"
#include "light/light.hpp"
#include "math/vector3.hpp"
#include "math/constants.hpp"


namespace pathtracer{

    // Also intersects a scene but quits early if one is found
    // that is closer than some distance
    Intersection Scene::intersect(
        const Ray& ray, real maxDistance) const {

        Intersection it = Intersection::NO_HIT;

        for(Instance* inst: m_instances){
            RayTracer::intersect(it, ray, inst);

            if(it.t() < maxDistance)
                return it;
        }

        return it;
    }


    Scene::Scene(const std::vector<Instance*>& instances,
        const std::vector<Light*>& lights) :
        m_instances{instances}, 
        m_lights{lights} {

        // These instances are now part of the scene, so we flag them
        for(Instance* inst: m_instances){
            inst->setInScene(true);
            m_bounds.extend(inst->getBoundingBox());
        }
    }


    const AxisAlignedBox& Scene::getBoundingBox() const {
        return m_bounds;
    }


    int Scene::instanceCount() const {
        return m_instances.size();
    }


    int Scene::lightCount() const {
        return m_lights.size();
    }


    const Instance* const Scene::instance(int index) const {
        assert((index >= 0 && index < m_instances.size())
            && "Index is out of bounds");

        return m_instances[index];
    }


    const Light* const Scene::light(int index) const {
        assert((index >= 0 && index < m_lights.size())
            && "Index is out of bounds");

        return m_lights[index];
    }


    // Expects ray this is in world coordinates.
    // Returns the closest hit.
    Intersection Scene::intersect(const Ray& ray) const {

        Intersection it = Intersection::NO_HIT;

        for(Instance* inst: m_instances){
            RayTracer::intersect(it, ray, inst);
        }

        return it;
    }


    bool Scene::visibility(
        const Vector3& origin, const Vector3& target) const {

        real distance = (origin - target).length();
        Vector3 direction = (target - origin).normalized();
        
        // We then add a small pad to avoid self intersection
        Ray ray(origin + direction * SHADOW_EPSILON, direction);
        real maxDistance = distance;

        Intersection it = intersect(
            ray, maxDistance - 2 * SHADOW_EPSILON);
        
        // If we don't find an intersection we return true
        if(!it)
            return true;

        // If we find an intersection and it is closer we return
        // false as well.
        if(it.t() < maxDistance - 2 * SHADOW_EPSILON){
            return false;
        }

        // Otherwise there are no obejcts in between origin and 
        // target, so we can return true.
        return true;
    }

}