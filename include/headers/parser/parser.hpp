#ifndef PATH_TRACER_PARSER_HPP
#define PATH_TRACER_PARSER_HPP

#include <nlohmann/json.hpp>
#include <unordered_map>
#include "../core/scene.hpp"
#include "../logger.hpp"
#include "../core/instance.hpp"
#include "../bsdf/bsdf.hpp"
#include "../bsdf/diffuseBsdf.hpp"
#include "../bsdf/principledBsdf.hpp"
#include "../light/light.hpp"
#include "../texture/texture.hpp"
#include "../shapes/sphere.hpp"
#include "../shapes/triangle.hpp"
#include "../emission/lambertianEmission.hpp"
#include "../light/areaLight.hpp"
#include "../shapes/mesh.hpp"
#include "../light/pointLight.hpp"
#include "../bsdf/mirrorBsdf.hpp"
#include "../camera/perspectiveCamera.hpp"
#include "../texture/color.hpp"
#include "../bsdf/dielectricBsdf.hpp"
#include "../renderer/renderer.hpp"
#include "../renderer/pathTracerRenderer.hpp"
#include "../integrator/aovIntegrator.hpp"
#include "../integrator/pathTracer.hpp"
#include "../integrator/pathTracerNee.hpp"
#include "../integrator/pathTracerNeeMis.hpp"
#include "../imageIo/imageIo.hpp"
#include "objLoader.hpp"
#include <fstream>

namespace pathtracer{

    using json = nlohmann::json;

    class Parser{


    private:

        
        static Vector3 parseVector3(const json& value){
            return Vector3(
                value[0],
                value[1],
                value[2]
            );
        }


        static Vector2 parseVector2(const json& value){
            return Vector2(
                value[0],
                value[1]
            );
        }


        static Vector2i parseVector2i(const json& value){
            return Vector2i(
                value[0],
                value[1]
            );
        }


        // Expects json of a texture
        static Texture* parseTexture(const json& texture){

            const json& value = texture["parameters"];
            if(texture["type"] == "texture"){

                std::string filename = value["file-name"];
                Texture::BorderMode bm;
                Texture::FilterMode fm;

                if(value["border-mode"] == "repeat"){
                    bm = Texture::BorderMode::REPEAT;
                }
                else if(value["border-mode"] == "clamp"){
                    bm = Texture::BorderMode::CLAMP;
                }
                else if(value["border-mode"] == "mirror"){
                    bm = Texture::BorderMode::MIRROR;
                }
                else{
                    LOG_ERROR("Invalid border-mode specified for: " + filename);
                    exit(1);
                }

                if(value["filter-mode"] == "nearest"){
                    fm = Texture::FilterMode::NEAREST;
                }
                else if (value["filter-mode"] == "bilinear"){
                    fm = Texture::FilterMode::BILINEAR;
                }
                else{
                    LOG_ERROR("Invalid filter-mode specified for: " + filename);
                    exit(1);
                }
                return new Texture(ImageIo::loadImage(filename), bm, fm);
            }
            else if(texture["type"] == "color"){
                Vector3 color = parseVector3(value["color"]);
                return new Color(color);
            }
            else{
                LOG_ERROR("Texture is not in a valid format.");
                exit(1);
            }
        }


        static Transform parseTransform(const json& value){
            // We assume the transform matrix is defined row by row,
            // But matrix is defined column by column
            return Transform(
                Matrix3(
                    Vector3(value[0], value[4], value[8]),
                    Vector3(value[1], value[5], value[9]),
                    Vector3(value[2], value[6], value[10])
                ), Vector3(value[3], value[7], value[11])
            );
        }


    public:

        static Renderer* parse(std::string filename){

            std::ifstream file(filename);

            if (!file.is_open()){
                LOG_ERROR("File could not be opened");
                exit(1);
            }

            json data = json::parse(file);

            std::unordered_map<std::string, Bsdf*> materials;
            std::unordered_map<std::string, Shape*> shapes;
            std::unordered_map<std::string, Instance*> instances;
            std::unordered_map<std::string, Light*> lights;
            Camera* camera;
            Integrator* integrator;
            Renderer* renderer; 

            // Only one copy needed
            Sphere* sphere = new Sphere();

       
            if(data.contains("camera")){
                    
                if(data["camera"]["type"] == "perspective"){

                    Vector3 eye = parseVector3(data["camera"]["parameters"]["look-at"]["eye"]);
                    Vector3 target = parseVector3(data["camera"]["parameters"]["look-at"]["target"]);
                    Vector3 up = parseVector3(data["camera"]["parameters"]["look-at"]["up"]);

                    Vector2 dim = parseVector2(data["camera"]["parameters"]["sensor-dimension"]);
                    real focalLength = data["camera"]["parameters"]["focal-length"];

                    camera = new PerspectiveCamera(dim.x(), dim.y(),
                        Camera::lookAt(eye, target, up),
                        focalLength);
                }
            }
            else{
                LOG_ERROR("No camera in: " + filename);
                exit(1);
            }


            if(data.contains("materials")){

                for(int i = 0; i < data["materials"].size(); i++){

                    std::string id = data["materials"][i]["id"];
                    
                    if(data["materials"][i]["type"] == "diffuse"){

                        Texture* texture = parseTexture(data["materials"][i]["parameters"]["albedo"]);

                        materials[id] = new DiffuseBsdf(texture);
                    }
                    else if(data["materials"][i]["type"] == "mirror"){
                        
                        real reflectance = data["materials"][i]["parameters"]["reflectance"];
                        materials[id] = new MirrorBsdf(reflectance);
                    }
                    else if(data["materials"][i]["type"] == "dielectric"){

                        Texture* ior = parseTexture(data["materials"][i]["parameters"]["ior"]);
                        Texture* reflectance = parseTexture(data["materials"][i]["parameters"]["reflectance"]);
                        Texture* transmittance = parseTexture(data["materials"][i]["parameters"]["transmittance"]);

                        materials[id] = new DielectricBsdf(ior, reflectance, transmittance);
                    }
                    else if(data["materials"][i]["type"] == "principled"){

                        Texture* baseColor = nullptr;
                        // Default values
                        Texture* roughness = new Color(Vector3(0.0));
                        Texture* metallic = new Color(Vector3(0.0));
                        Texture* specular = new Color(Vector3(0.0));

                        baseColor = parseTexture(data["materials"][i]["parameters"]["base-color"]);

                        if(data["materials"][i]["parameters"].contains("roughness")){
                            roughness = parseTexture(data["materials"][i]["parameters"]["roughness"]);
                        }

                        if(data["materials"][i]["parameters"].contains("metallic")){
                            metallic = parseTexture(data["materials"][i]["parameters"]["metallic"]);
                        }

                        if(data["materials"][i]["parameters"].contains("specular")){
                            specular = parseTexture(data["materials"][i]["parameters"]["specular"]);
                        }

                        materials[id] = new PrincipledBsdf(baseColor, roughness, metallic, specular);
                    }

                }

            }

            if(data.contains("instances")){
                
                for(int i = 0; i < data["instances"].size(); i++){

                    json& inst = data["instances"][i];
                    std::string id = inst["id"];

                    std::string materialID = inst["material"];
                    if(materials.find(materialID) == materials.end()){
                        LOG_ERROR("Could not find specified material: " + materialID);
                        exit(1);
                    }

                    Emission* emission = nullptr;
                    if(inst.contains("emission")){
                        if(inst["emission"]["type"] == "lambertian"){
                            emission = new LambertianEmission(
                                parseVector3(inst["emission"]["parameters"]["power"]));
                        }
                    }

                    Transform transform = Transform::IDENTITY;
                    if(inst["parameters"].contains("transform")){
                        transform = parseTransform(inst["parameters"]["transform"]);
                    }

                    Texture* normalMap = nullptr;
                    if(inst["parameters"].contains("normal-map")){
                        normalMap = parseTexture(inst["parameters"]["normal-map"]);
                    }

                    Texture* alphaMask = nullptr;
                    if(inst["parameters"].contains("alpha-mask")){
                        alphaMask = parseTexture(inst["parameters"]["alpha-mask"]);
                    }

                    if(inst["type"] == "sphere"){

                        Instance* instance = new Instance(sphere, nullptr,
                            nullptr, materials[materialID], emission, transform);

                        instances[id] = instance;
                    }


                    if(inst["type"] == "mesh"){
    
                        Mesh* mesh;

                        if(inst["parameters"]["vertex-data"]["format"] == "raw-array"){
                            std::vector<Triangle> triangles;
                            triangles.reserve(inst["parameters"]["vertex-data"]["parameters"]["vertices"].size());

                            // We need to loop over every triangle
                            for(int j = 0; j < inst["parameters"]["vertex-data"]["parameters"]["vertices"].size(); j++){

                                triangles.emplace_back(Triangle(
                                    parseVector3(inst["parameters"]["vertex-data"]["parameters"]["vertices"][j][0]),
                                    parseVector3(inst["parameters"]["vertex-data"]["parameters"]["vertices"][j][1]),
                                    parseVector3(inst["parameters"]["vertex-data"]["parameters"]["vertices"][j][2])
                                ));
                            }

                            mesh = new Mesh(triangles);
                        }
                        else if(inst["parameters"]["vertex-data"]["format"] == "obj"){
                            std::string filename = inst["parameters"]["vertex-data"]["parameters"]["file-name"];
                            mesh = ObjLoader::loadMesh(filename);
                        }

                        instances[id] = new Instance(mesh, alphaMask,
                            normalMap, materials[materialID], emission, transform);
                    }
                }
            }


            if(data.contains("lights")){

                for(int i = 0; i < data["lights"].size(); i++){

                    std::string id = data["lights"][i]["id"];
                    
                    if(data["lights"][i]["type"] == "area"){

                        std::string instID = data["lights"][i]["parameters"]["instance"];

                        // Instance should have emission
                        if(instances.find(instID) == instances.end()){
                            LOG_ERROR("Could not find specified emissive instance: " + instID);
                            exit(1);
                        }
                        if(!instances[instID]->emission()){
                            LOG_ERROR("Instance is not emissive: " + instID);
                            exit(1);
                        }
                        lights[id] = new AreaLight(instances[instID]);

                    }
                    else if(data["lights"][i]["type"] == "point"){
                        Vector3 power = parseVector3(data["lights"][i]["parameters"]["power"]);
                        Vector3 pos = parseVector3(data["lights"][i]["parameters"]["position"]);
                        lights[id] = new PointLight(pos, power);
                    }
                }  

            }


            // Vector creation
            std::vector<Instance*> instanceVector;
            instanceVector.reserve(instances.size());

            for (const auto& [id, instance] : instances) {
                instanceVector.push_back(instance);
            }

            std::vector<Light*> lightVector;
            lightVector.reserve(lights.size());

            for (const auto& [id, light] : lights) {
                lightVector.push_back(light);
            }

            Scene* scene = new Scene(instanceVector, lightVector);


            if(data.contains("renderer")){

                Vector2i resolution = parseVector2i(data["renderer"]["parameters"]["pixel-resolution"]);
                
                if(data["renderer"]["integrator"]["type"] == "aov-integrator"){
                    std::string var = data["renderer"]["integrator"]["parameters"]["render-variable"];
                    if(var == "normal"){
                        integrator = new AovIntegrator(AovIntegrator::RenderVariable::NORMAL);
                    }
                }
                else if(data["renderer"]["integrator"]["type"] == "path-tracer"){
                    int maxDepth = data["renderer"]["integrator"]["parameters"]["max-depth"];
                    integrator = new PathTracer(maxDepth);
                }
                else if(data["renderer"]["integrator"]["type"] == "path-tracer-nee"){
                    int maxDepth = data["renderer"]["integrator"]["parameters"]["max-depth"];
                    integrator = new PathTracerNee(maxDepth);
                }
                else if(data["renderer"]["integrator"]["type"] == "path-tracer-nee-mis"){
                    int maxDepth = data["renderer"]["integrator"]["parameters"]["max-depth"];
                    integrator = new PathTracerNeeMis(maxDepth);
                }

                if(data["renderer"]["type"] == "path-tracer-renderer"){
                    int sampleCount = data["renderer"]["parameters"]["sample-count"];
                    renderer = new PathTracerRenderer(
                        resolution.x(), resolution.y(), camera, scene, 
                        integrator, sampleCount);
                }
                else if(data["renderer"]["type"] == "renderer"){
                    renderer = new Renderer(resolution.x(), resolution.y(), 
                        camera, scene, integrator);
                }
            }
            else{
                LOG_ERROR("No renderer in: " + filename);
                exit(1);
            }
            
            return renderer;
        }

    };

}

#endif