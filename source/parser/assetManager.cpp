
#include "parser/assetManager.hpp"
#include "bsdf/bsdf.hpp"
#include "shapes/shape.hpp"
#include "texture/texture.hpp"
#include "texture/mipMap.hpp"

namespace pathtracer {

    AssetManager::AssetManager() = default;

    AssetManager::~AssetManager() = default;

    AssetManager::AssetManager(AssetManager&&) = default;

    AssetManager& AssetManager::operator=(AssetManager&&) = default;

    void AssetManager::addMaterial(std::unique_ptr<Bsdf> material) {
        m_materials.push_back(std::move(material));
    }

    void AssetManager::addShape(std::unique_ptr<Shape> shape) {
        m_shapes.push_back(std::move(shape));
    }

    void AssetManager::addTexture(std::unique_ptr<Texture> texture) {
        m_texture.push_back(std::move(texture));
    }

    void AssetManager::addMipMap(std::unique_ptr<MipMap> mipmap) {
        m_mipmaps.push_back(std::move(mipmap));
    }

}