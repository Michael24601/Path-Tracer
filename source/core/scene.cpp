
#include "core/scene.hpp"
#include <cassert>
#include <utility>
#include "core/instance.hpp"
#include "core/ray.hpp"
#include "intersection/rayTracer.hpp"
#include "intersection/intersection.hpp"
#include "light/light.hpp"
#include "math/vector3.hpp"
#include "math/constants.hpp"
#include "logger.hpp"


namespace pathtracer{

    // Also intersects a scene but quits early if one is found
    // that is closer than some distance
    Intersection Scene::intersect(const Ray& ray, real maxDistance) const {

        Intersection it = Intersection::NO_HIT;

        for(auto& inst: m_instances){
            RayTracer::intersect(it, ray, inst.get());

            if(it && !std::isfinite(maxDistance)){
                return it;
            }
            else if(it.t() < maxDistance){
                return it;
            }
        }

        return it;
    }


    Scene::Scene(AssetManager assetManager,
        std::vector<std::unique_ptr<Instance>> instances,
        std::vector<std::unique_ptr<Light>> lights)
        : m_assetManager(std::move(assetManager)),
        m_instances(std::move(instances)),
        m_lights(std::move(lights)) {

        // These instances are now part of the scene, so we flag them
        for(auto& inst: m_instances){
            inst->setInScene(true);
            m_bounds.extend(inst->getBoundingBox());
        }

        // We can scale it up to avoid numerical errors
        m_bounds.scale(1.1);
    }


    Scene::~Scene() = default;


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

        return m_instances[index].get();
    }


    const Light* const Scene::light(int index) const {
        assert((index >= 0 && index < m_lights.size())
            && "Index is out of bounds");

        return m_lights[index].get();
    }


    // Expects ray this is in world coordinates.
    // Returns the closest hit.
    Intersection Scene::intersect(const Ray& ray) const {

        Intersection it = Intersection::NO_HIT;

        for(auto& inst: m_instances){
            RayTracer::intersect(it, ray, inst.get());
        }

        return it;
    }


    bool Scene::visibility(const Vector3& origin, const Vector3& target) const {

        // This function should NOT be used for directional lights, or
        // any lights, as they can have infinitely far away points.
        if(!target.isFinite()){
            LOG_ERROR("Infinitely far away point should not be using this visibility function.");
            throw std::runtime_error("");
        }

        real distance = (origin - target).length();
        Vector3 direction = (target - origin).normalized();
        
        return visibility(origin, direction, distance);
    }


    // For lights, especially directional, it's better to use
    // this version, with a direction and distance
    bool Scene::visibility(const Vector3& origin, const Vector3& direction,
        real distance) const {
        
        // We then add a small pad to avoid self intersection
        Ray ray(origin + direction * SHADOW_EPSILON, direction);
        real maxDistance = distance;

        Intersection it = intersect(ray, maxDistance - 2 * SHADOW_EPSILON);
        
        // If we don't find an intersection we return true
        if(!it){
            return true;
        }

        if(!std::isfinite(maxDistance)){
            return false;
        }

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