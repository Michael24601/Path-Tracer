#include "parser/parser.hpp"
#include <fstream>
#include <string>
#include <unordered_map>
#include <nlohmann/json.hpp>
#include "core/instance.hpp"
#include "core/scene.hpp"
#include "core/transform.hpp"
#include "bsdf/bsdf.hpp"
#include "bsdf/diffuseBsdf.hpp"
#include "bsdf/principledBsdf.hpp"
#include "bsdf/mirrorBsdf.hpp"
#include "bsdf/roughConductorBsdf.hpp"
#include "bsdf/dielectricBsdf.hpp"
#include "camera/perspectiveCamera.hpp"
#include "emission/lambertianEmission.hpp"
#include "image/imageIo.hpp"
#include "integrator/aovIntegrator.hpp"
#include "integrator/pathTracer.hpp"
#include "integrator/pathTracerNee.hpp"
#include "integrator/pathTracerNeeMis.hpp"
#include "integrator/pathTracerGuided.hpp"
#include "integrator/pathTracerNeeMisGuided.hpp"
#include "integrator/pathTracerNeeGuided.hpp"
#include "integrator/photonMappingIntegrator.hpp"
#include "integrator/pathTracerSms.hpp"
#include "integrator/pathTracerNeeSms.hpp"
#include "light/light.hpp"
#include "light/areaLight.hpp"
#include "light/pointLight.hpp"
#include "light/directionalLight.hpp"
#include "logger.hpp"
#include "math/matrix3.hpp"
#include "math/vector2.hpp"
#include "math/vector2i.hpp"
#include "math/vector3.hpp"
#include "renderer/renderer.hpp"
#include "renderer/pathTracerRenderer.hpp"
#include "renderer/pathGuidingRenderer.hpp"
#include "renderer/photonMappingRenderer.hpp"
#include "shapes/mesh.hpp"
#include "shapes/sphere.hpp"
#include "shapes/triangle.hpp"
#include "texture/texture.hpp"
#include "texture/leanMap.hpp"
#include "texture/mipMap.hpp"
#include "texture/color.hpp"
#include "parser/objLoader.hpp"
#include "texture/leanMap.hpp"
#include "parser/assetManager.hpp"


namespace pathtracer{


    using json = nlohmann::json;


    const json& require(const json& object, const std::string& key) {
        if(!object.is_object() || !object.contains(key)){
            LOG_ERROR("Parameter does not exist: " + key);
            throw std::runtime_error("");
        }
        return object[key];
    }


    std::vector<std::vector<Vector3>> parseImage(const std::string& filename) {

        std::vector<std::vector<Vector3>> image;

        if(!ImageIo::loadImage(filename, image)){
            LOG_ERROR("Could not load image: " + filename);
            throw std::runtime_error("");
        }

        if(image.empty() || image[0].empty()){
            LOG_ERROR("Image is empty: " + filename);
            throw std::runtime_error("");
        }

        return image;
    }


    real parseReal(const json& object, const std::string& key) {
        const json& value = require(object, key);
        if(!value.is_number()){
            LOG_ERROR("Parameter is not a number: " + key);
            throw std::runtime_error("");
        }
        return value.get<real>();
    }


    int parseInt(const json& object, const std::string& key) {
        const json& value = require(object, key);
        if(!value.is_number_integer()){
            LOG_ERROR("Parameter is not an integer: " + key);
            throw std::runtime_error("");
        }
        return value.get<int>();
    }


    bool parseBool(const json& object, const std::string& key) {
        const json& value = require(object, key);
        if(!value.is_boolean()){
            LOG_ERROR("Parameter is not a boolean: " + key);
            throw std::runtime_error("");
        }
        return value.get<bool>();
    }


    std::string parseString(const json& object, const std::string& key) {
        const json& value = require(object, key);
        if(!value.is_string()){
            LOG_ERROR("Parameter is not a string: " + key);
            throw std::runtime_error("");
        }
        return value.get<std::string>();
    }


    Vector3 parseVector3(const json& value) {
        return Vector3(
            value[0],
            value[1],
            value[2]
        );
    }


    Vector3 parseVector3(const json& object, const std::string& key) {
        return parseVector3(require(object, key));
    }


    Vector2 parseVector2(const json& value) {
        return Vector2(
            value[0],
            value[1]
        );
    }


    Vector2 parseVector2(const json& object, const std::string& key) {
        return parseVector2(require(object, key));
    }


    Vector2i parseVector2i(const json& value) {
        return Vector2i(
            value[0],
            value[1]
        );
    }


    Vector2i parseVector2i(const json& object, const std::string& key) {
        return parseVector2i(require(object, key));
    }


    Texture* parseTexture(const json& texture,
        std::unordered_map<std::string, Texture*>& textures,
        std::vector<Color*>& colors,
        bool linearize = false, bool isNormalMap = false, bool mipMap = false) {

        const json& value = require(texture, "parameters");
        std::string type = parseString(texture, "type");

        if(type == "texture"){

            std::string filename = parseString(value, "file-name");
            std::string borderMode = parseString(value, "border-mode");
            std::string filterMode = parseString(value, "filter-mode");

            // If the texture already exists, we don't need to recreate it
            auto it = textures.find(filename);
            if (it != textures.end()) {
                return it->second;
            }

            BorderMode bm;
            FilterMode fm;

            if(borderMode == "repeat"){
                bm = BorderMode::REPEAT;
            }
            else if(borderMode == "clamp"){
                bm = BorderMode::CLAMP;
            }
            else if(borderMode == "mirror"){
                bm = BorderMode::MIRROR;
            }
            else{
                LOG_ERROR("Invalid border-mode specified for: " + filename);
                throw std::runtime_error("");
            }

            if(filterMode == "nearest"){

                if(isNormalMap){
                    std::string message = "Normal maps should not use nearest filtering "
                        "as differentiating the normal map to get DNDU and DNDV "
                        "requires a step size smaller than a pixel for most resolutions.";
                    LOG_WARNING(message);
                }

                fm = FilterMode::NEAREST;
            }
            else if(filterMode == "bilinear"){
                fm = FilterMode::BILINEAR;
            }
            else{
                LOG_ERROR("Invalid filter-mode specified for: " + filename);
                throw std::runtime_error("");
            }

            Texture* newTexture = new Texture(parseImage(filename), bm, fm, 
                linearize, isNormalMap);

            textures[filename] = newTexture;
            return newTexture;
        }
        else if(type == "color"){
            
            Vector3 color = parseVector3(value, "color");
            
            // We have to store it
            Color* newColor = new Color(color);

            colors.push_back(newColor);
            return newColor;
        }
        else{
            LOG_ERROR("Texture is not in a valid format.");
            throw std::runtime_error("");
        }
    }


    Transform parseTransform(const json& value) {
        return Transform(
            Matrix3(
                Vector3(value[0], value[4], value[8]),
                Vector3(value[1], value[5], value[9]),
                Vector3(value[2], value[6], value[10])
            ),
            Vector3(value[3], value[7], value[11])
        );
    }


    std::pair<Scene*, Renderer*> Parser::parse(const std::string& filename) {

        std::ifstream file(filename);

        if (!file.is_open()){
            LOG_ERROR("File could not be opened");
            throw std::runtime_error("");
        }

        json data = json::parse(file);

        // These use the source file path as id to check for duplicates
        std::unordered_map<std::string, MipMap*> mipmaps;
        std::unordered_map<std::string,Texture*> textures;
        std::vector<Color*> colors;
        std::unordered_map<std::string, Shape*> shapes;

        std::unordered_map<std::string, Bsdf*> materials;
        Sphere* sphere = new Sphere();
    
        std::unordered_map<std::string, Instance*> instances;
        std::unordered_map<std::string, Light*> lights;

        Camera* camera = nullptr;
        Integrator* integrator = nullptr;
        
        Renderer* renderer = nullptr;
        
        if(data.contains("camera")){

            const json& cameraJson = data["camera"];

            if(parseString(cameraJson, "type") == "perspective"){

                const json& params = require(cameraJson, "parameters");
                const json& lookAt = require(params, "look-at");

                Vector3 eye = parseVector3(lookAt, "eye");
                Vector3 target = parseVector3(lookAt, "target");
                Vector3 up = parseVector3(lookAt, "up");
                Vector2 dim = parseVector2(params, "sensor-dimension");
                real focalLength = parseReal(params, "focal-length");

                camera = new PerspectiveCamera(
                    dim.x(),
                    dim.y(),
                    Camera::lookAt(eye, target, up),
                    focalLength);
            }
            else{
                LOG_ERROR("Camera type is not supported in: " + filename);
                throw std::runtime_error("");
            }
        }
        else{
            LOG_ERROR("No camera in: " + filename);
            throw std::runtime_error("");
        }

        if(data.contains("materials")){

            for(int i = 0; i < data["materials"].size(); i++){

                const json& material = data["materials"][i];

                std::string id = parseString(material, "id");
                std::string type = parseString(material, "type");
                const json& params = require(material, "parameters");

                if(type == "diffuse"){
                    
                    Texture* texture = parseTexture(require(params, "albedo"), 
                        textures, colors, true);

                    materials[id] = new DiffuseBsdf(texture);
                }
                else if(type == "mirror"){

                    Texture* reflectance = parseTexture(require(params, "reflectance"),
                        textures, colors);

                    materials[id] = new MirrorBsdf(reflectance);
                }
                else if(type == "rough-conductor"){

                    Texture* reflectance = parseTexture(require(params, "reflectance"),
                         textures, colors);
                    Texture* roughness = parseTexture(require(params, "roughness"),
                         textures, colors);

                    materials[id] = new RoughConductorBsdf(reflectance, roughness);
                }
                else if(type == "dielectric"){

                    Texture* ior = parseTexture(require(params, "ior"),
                        textures, colors);
                    Texture* reflectance = parseTexture(require(params, "reflectance"),
                        textures, colors);
                    Texture* transmittance = parseTexture(require(params, "transmittance"),
                        textures, colors);

                    materials[id] = new DielectricBsdf(ior, reflectance, 
                        transmittance);
                }
                else if(type == "principled"){


                    Texture* baseColor = parseTexture(require(params, "base-color"),
                        textures, colors);

                    Texture* roughness = parseTexture(require(params, "roughness"),
                        textures, colors);

                    Texture* metallic = parseTexture(require(params, "metallic"),
                        textures, colors);
                    
                    Texture* specular = parseTexture(require(params, "specular"),
                        textures, colors);

                    materials[id] = new PrincipledBsdf(baseColor, roughness, 
                        metallic, specular);
                }
            }
        }


        if(data.contains("instances")){

            for(int i = 0; i < data["instances"].size(); i++){

                const json& inst = data["instances"][i];

                std::string id = parseString(inst, "id");
                std::string materialID = parseString(inst, "material");
                std::string instType = parseString(inst, "type");
                const json& params = require(inst, "parameters");

                if(materials.find(materialID) == materials.end()){
                    LOG_ERROR(
                        "Could not find specified material: " + materialID);

                    throw std::runtime_error("");
                }

                Emission* emission = nullptr;

                if(inst.contains("emission")){
                    const json& emissionJson = inst["emission"];

                    if(parseString(emissionJson, "type") == "lambertian"){
                        emission = new LambertianEmission(
                            parseVector3(
                                require(emissionJson, "parameters"), "power"
                            )
                        );
                    }
                }

                Transform transform = Transform::IDENTITY;

                if(params.contains("transform")){
                    transform = parseTransform(params["transform"]);
                }

                bool isCausticReceiver = false;
                if(params.contains("is-caustic-receiver")){
                    isCausticReceiver = parseBool(params, "is-caustic-receiver");
                }

                Texture* normalMap = nullptr;
                MipMap* lean = nullptr;

                if(params.contains("normal-map")){
                    normalMap = parseTexture(params["normal-map"],
                        textures, colors, false, true, true);

                    std::string filename = parseString(
                        require(params["normal-map"], "parameters"), "file-name");

                    // If lean map already exists, we can reuse it
                    auto it = mipmaps.find(filename);
                    if (it != mipmaps.end()) {
                        lean = it->second;
                    }
                    else {
                        lean = LeanMap::constructLeanMap(normalMap);
                        mipmaps[filename] = lean;
                    }
                }

                Texture* alphaMask = nullptr;

                if(params.contains("alpha-mask")){
                    alphaMask = parseTexture(params["alpha-mask"],
                        textures, colors);
                }

                // Just a raw pointer to shape, the actual resource
                // is stored in the shapes vector.
                const Shape* shape = nullptr;

                if(instType == "sphere"){
                    shape = sphere;
                }
                else if(instType == "mesh"){

                    const json& vertexData = require(params, "vertex-data");
                    std::string format = parseString(vertexData, "format");
                    const json& vertexParams = require(vertexData, "parameters");

                    if(format == "raw-array"){

                        const json& vertices = require(vertexParams, "vertices");

                        std::vector<Triangle> triangles;
                        triangles.reserve(vertices.size());

                        for(int j = 0; j < vertices.size(); j++){
                            triangles.emplace_back(Triangle(
                                parseVector3(vertices[j][0]),
                                parseVector3(vertices[j][1]),
                                parseVector3(vertices[j][2])
                            ));
                        }

                        Mesh* mesh = new Mesh(triangles);
                        shape = mesh;
                        shapes[id] = mesh;
                    }
                    else if(format == "obj"){

                        std::string objFilename =
                            parseString(vertexParams, "file-name");

                        // Checks if duplicated
                        auto it = shapes.find(objFilename);
                        if (it != shapes.end()) {
                            shape = it->second;
                        }
                        else {
                            Mesh* mesh = ObjLoader::loadMesh(objFilename);
                            shape = mesh;

                            // We use path as id to check for duplicates
                            shapes[objFilename] = mesh;
                        }
                    }
                    else{
                        LOG_ERROR("Invalid vertex-data format for instance: " + id);
                        throw std::runtime_error("");
                    }

                }
                else{
                    LOG_ERROR("Instance does not have a valid shape.");
                    throw std::runtime_error("");
                }

                Instance* instance = new Instance(
                    shape, alphaMask, normalMap, lean, 
                    materials[materialID], emission, transform
                );

                instance->setCausticReceiver(isCausticReceiver);
                instances[id] = instance;
            }
        }

        if(data.contains("lights")){

            for(int i = 0; i < data["lights"].size(); i++){

                const json& light = data["lights"][i];

                std::string id = parseString(light, "id");
                std::string type = parseString(light, "type");
                const json& params = require(light, "parameters");

                if(type == "area"){

                    std::string instID = parseString(params, "instance");

                    if(instances.find(instID) == instances.end()){
                        LOG_ERROR(
                            "Could not find specified emissive instance: "
                            + instID);

                        throw std::runtime_error("");
                    }

                    if(!instances[instID]->emission()){
                        LOG_ERROR("Instance is not emissive: " + instID);
                        throw std::runtime_error("");
                    }

                    lights[id] = new AreaLight(instances[instID]);
                }
                else if(type == "point"){

                    Vector3 power = parseVector3(params, "power");
                    Vector3 pos = parseVector3(params, "position");

                    lights[id] = new PointLight(pos, power);
                }
                else if(type == "directional"){

                    Vector3 power = parseVector3(params, "power");
                    Vector3 dir = parseVector3(params, "direction");

                    lights[id] = new DirectionalLight(dir, power);
                }
            }
        }


        std::vector<Instance*> instanceVector;
        instanceVector.reserve(instances.size());

        for (auto& [id, instance] : instances) {
            instanceVector.push_back(instance);
        }

        std::vector<Light*> lightVector;
        lightVector.reserve(lights.size());

        for (auto& [id, light] : lights) {
            lightVector.push_back(light);
        }


        // The assets to preserve
        AssetManager* assetManager = new AssetManager();

        for (auto& [id, mipmap] : mipmaps)
            assetManager->addMipMap(mipmap);
        for (auto& [id, texture] : textures)
            assetManager->addTexture(texture);
        for (auto& color : colors)
            assetManager->addTexture(color);
        for (auto& [id, shape] : shapes)
            assetManager->addShape(shape);
        for (auto& [id, material] : materials)
            assetManager->addMaterial(material);
        assetManager->addShape(sphere);


        Scene* scene = new Scene(assetManager, instanceVector, lightVector);

        if(data.contains("renderer")){

            const json& rendererJson = data["renderer"];
            const json& rendererParams = require(rendererJson, "parameters");
            const json& integratorJson = require(rendererJson, "integrator");

            Vector2i resolution = parseVector2i(rendererParams, "pixel-resolution");

            std::string integratorType = parseString(integratorJson, "type");
            const json& ip = require(integratorJson, "parameters");

            if(integratorType == "aov-integrator"){

                std::string var = parseString(ip, "render-variable");

                if(var == "shading-normal"){
                    integrator = new AovIntegrator(
                        AovIntegrator::RenderVariable::SHADING_NORMAL);
                }
                else if(var == "geometry-normal"){
                    integrator = new AovIntegrator(
                        AovIntegrator::RenderVariable::GEOMETRY_NORMAL);
                }
                else if(var == "lean-normal"){
                    integrator = new AovIntegrator(
                        AovIntegrator::RenderVariable::LEAN_NORMAL);
                }
                else if(var == "mipmap-normal"){
                    integrator = new AovIntegrator(
                        AovIntegrator::RenderVariable::MIPMAP_NORMAL);
                }
                else if(var == "dpdu"){
                    integrator = new AovIntegrator(
                        AovIntegrator::RenderVariable::DPDU);
                }
                else if(var == "dpdv"){
                    integrator = new AovIntegrator(
                        AovIntegrator::RenderVariable::DPDV);
                }
                else if(var == "dndu"){
                    integrator = new AovIntegrator(
                        AovIntegrator::RenderVariable::DNDU);
                }
                else if(var == "dndv"){
                    integrator = new AovIntegrator(
                        AovIntegrator::RenderVariable::DNDV);
                }
                else if(var == "s"){
                    integrator = new AovIntegrator(
                        AovIntegrator::RenderVariable::S);
                }
                else if(var == "dsdu"){
                    integrator = new AovIntegrator(
                        AovIntegrator::RenderVariable::DSDU);
                }
                else if(var == "dsdv"){
                    integrator = new AovIntegrator(
                        AovIntegrator::RenderVariable::DSDV);
                }
                else if(var == "dtdu"){
                    integrator = new AovIntegrator(
                        AovIntegrator::RenderVariable::DTDU);
                }
                else if(var == "dtdv"){
                    integrator = new AovIntegrator(
                        AovIntegrator::RenderVariable::DTDV);
                }
                else{
                    LOG_ERROR("Invalid render-variable: " + var);
                    throw std::runtime_error("");
                }
            }
            else if(integratorType == "path-tracer"){
                integrator = new PathTracer(parseInt(ip, "max-depth"));
            }
            else if(integratorType == "path-tracer-nee"){
                integrator = new PathTracerNee(parseInt(ip, "max-depth"));
            }
            else if(integratorType == "path-tracer-nee-mis"){
                integrator = new PathTracerNeeMis(parseInt(ip, "max-depth"));
            }
            else if(integratorType == "path-tracer-guided"){
                integrator = new PathTracerGuided(parseInt(ip, "max-depth"));
            }
            else if(integratorType == "path-tracer-nee-guided"){
                integrator = new PathTracerNeeGuided(parseInt(ip, "max-depth"));
            }
            else if(integratorType == "path-tracer-nee-mis-guided"){
                integrator = new PathTracerNeeMisGuided(parseInt(ip, "max-depth"));
            }
            else if(integratorType == "path-tracer-sms"){

                int maxDepth = parseInt(ip, "max-depth");
                bool useHalfVector = parseBool(ip, "use-halfvector");
                bool biased = parseBool(ip, "biased");
                bool twoStage = parseBool(ip, "two-stage");

                integrator =
                    new PathTracerSms(maxDepth, useHalfVector, biased, twoStage, scene);
            }
            else if(integratorType == "path-tracer-nee-sms"){

                int maxDepth = parseInt(ip, "max-depth");
                bool useHalfVector = parseBool(ip, "use-halfvector");
                bool biased = parseBool(ip, "biased");
                bool twoStage = parseBool(ip, "two-stage");

                integrator =
                    new PathTracerNeeSms(maxDepth, useHalfVector, biased, twoStage, scene);
            }
            else if(integratorType == "photon-mapping-integrator"){

                int maxDepth = parseInt(ip, "max-depth");
                real kernelRadius = parseReal(ip, "kernel-radius");

                KernelType type;
                std::string var = parseString(ip, "kernel-type");

                if(var == "uniform"){
                    type = KernelType::UNIFORM;
                }
                else if(var == "epanechnikov"){
                    type = KernelType::EPANECHNIKOV;
                }
                else{
                    LOG_ERROR("Invalid kernel-type: " + var);
                    throw std::runtime_error("");
                }

                integrator = new PhotonMappingIntegrator(maxDepth,
                    kernelRadius, type);
            }
            else{
                LOG_ERROR("No integrator found.");
                throw std::runtime_error("");
            }

            int messageCount = 10;
            if(rendererParams.contains("message-count")){
                messageCount = parseInt(rendererParams, "message-count");
            }

            std::string rendererType = parseString(rendererJson, "type");

            if(rendererType == "path-tracer-renderer"){

                int sampleCount = parseInt(rendererParams, "sample-count");

                renderer = new PathTracerRenderer(
                    resolution.x(), resolution.y(),
                    camera, scene, integrator, sampleCount, messageCount
                );
            }
            else if(rendererType == "photon-mapping-renderer"){

                int maxDepth = parseInt(rendererParams, "max-depth");
                int threshold = parseInt(rendererParams, "threshold");
                int photonCount = parseInt(rendererParams, "photon-count");

                renderer = new PhotonMappingRenderer(
                    resolution.x(), resolution.y(),
                    camera, scene, integrator, photonCount, threshold, maxDepth,
                    messageCount
                );
            }
            else if(rendererType == "renderer"){

                renderer = new Renderer(
                    resolution.x(), resolution.y(), camera, scene,
                    integrator, messageCount
                );
            }
            else if(rendererType == "path-guiding-renderer"){

                int renderSamples = parseInt(rendererParams, "sample-count");

                int firstIterationSamples = 4;
                if(rendererParams.contains("first-iteration-sample-count")){
                    firstIterationSamples =
                        parseInt(rendererParams, "first-iteration-sample-count");
                }

                int iterationCount = parseInt(rendererParams, "iteration-count");
                int c = parseInt(rendererParams, "spatial-subdivision-c");
                real alpha = parseReal(rendererParams, "mis-alpha");

                renderer = new PathGuidingRenderer(
                    resolution.x(), resolution.y(),
                    camera, scene, integrator,
                    iterationCount, renderSamples, c, alpha,
                    firstIterationSamples, messageCount
                );
            }
            else{
                LOG_ERROR("No renderer found.");
                throw std::runtime_error("");
            }
        }
        else{
            LOG_ERROR("No renderer in: " + filename);
            throw std::runtime_error("");
        }

        return std::make_pair(scene, renderer);
    }

}