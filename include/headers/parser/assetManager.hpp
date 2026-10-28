
#ifndef PATH_TRACER_ASSET_MANAGER_HPP
#define PATH_TRACER_ASSET_MANAGER_HPP

#include <memory>
#include <vector>

namespace pathtracer{

    class Shape;
    class Bsdf;
    class Texture;
    class MipMap;

    class AssetManager{

    private:
        
        // The manager owns the materials and shapes... which it doesn't
        // use directly but is responsible for managing (referenced by
        // instances)
        std::vector<std::unique_ptr<Bsdf>> m_materials;
        std::vector<std::unique_ptr<Shape>> m_shapes;
        std::vector<std::unique_ptr<Texture>> m_texture;
        std::vector<std::unique_ptr<MipMap>> m_mipmaps;
        
    public:

        AssetManager();

        ~AssetManager();

        AssetManager(AssetManager&&);
        
        AssetManager& operator=(AssetManager&&);

        void addMaterial(std::unique_ptr<Bsdf> material);
        void addShape(std::unique_ptr<Shape> shape);
        void addTexture(std::unique_ptr<Texture> texture);
        void addMipMap(std::unique_ptr<MipMap> mipmap);

    };

}

#endif