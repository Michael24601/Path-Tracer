#ifndef PATH_TRACER_SCENE_HPP
#define PATH_TRACER_SCENE_HPP

#include <vector>
#include <memory>
#include "bvh/axisAlignedBox.hpp"
#include "parser/assetManager.hpp"

namespace pathtracer{

    class Instance;
    class Light;
    class Ray;
    class Intersection;
    class Vector3;
    class Scene{

    private:

        std::vector<std::unique_ptr<Instance>> m_instances;
        std::vector<std::unique_ptr<Light>> m_lights;
        AssetManager m_assetManager;

        AxisAlignedBox m_bounds;

        // Also intersects a scene but quits early if one is found
        // that is closer than some distance
        Intersection intersect(const Ray& ray, real maxDistance) const;


    public:

        Scene(AssetManager assetManager,
            std::vector<std::unique_ptr<Instance>> instances,
            std::vector<std::unique_ptr<Light>> lights);


        ~Scene();


        const AxisAlignedBox& getBoundingBox() const;


        int instanceCount() const;


        int lightCount() const;


        const Instance* const instance(int index) const;


        const Light* const light(int index) const;


        // Expects ray this is in world coordinates.
        // Returns the closest hit.
        Intersection intersect(const Ray& ray) const;


        bool visibility(const Vector3& origin, const Vector3& target) const;

        
        bool visibility(const Vector3& origin, const Vector3& direction,
            real distance) const;

    };

}

#endif