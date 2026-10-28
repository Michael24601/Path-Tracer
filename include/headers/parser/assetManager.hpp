
#ifndef PATH_TRACER_ASSET_MANAGER_HPP
#define PATH_TRACER_ASSET_MANAGER_HPP

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
        std::vector<Bsdf*> m_materials;
        std::vector<Shape*> m_shapes;
        std::vector<Texture*> m_texture;
        std::vector<MipMap*> m_mipmaps;
        
    public:

        AssetManager();

        ~AssetManager();

        void addMaterial(Bsdf* material);
        void addShape(Shape* shape);
        void addTexture(Texture* texture);
        void addMipMap(MipMap* mipmap);

    };

}

#endif