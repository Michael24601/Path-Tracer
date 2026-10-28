
#include "parser/assetManager.hpp"
#include "bsdf/bsdf.hpp"
#include "shapes/shape.hpp"
#include "texture/texture.hpp"
#include "texture/mipMap.hpp"

namespace pathtracer {

    AssetManager::AssetManager(){}

    AssetManager::~AssetManager(){
        for(int i = 0; i < m_materials.size(); i++){
            delete m_materials[i];
        }

        for(int i = 0; i < m_mipmaps.size(); i++){
            delete m_mipmaps[i];
        }

        for(int i = 0; i < m_shapes.size(); i++){
            delete m_shapes[i];
        }

        for(int i = 0; i < m_texture.size(); i++){
            delete m_texture[i];
        }
    }


    void AssetManager::addMaterial(Bsdf* material) {
        m_materials.push_back(material);
    }

    void AssetManager::addShape(Shape* shape) {
        m_shapes.push_back(shape);
    }

    void AssetManager::addTexture(Texture* texture) {
        m_texture.push_back(texture);
    }

    void AssetManager::addMipMap(MipMap* mipmap) {
        m_mipmaps.push_back(mipmap);
    }

}