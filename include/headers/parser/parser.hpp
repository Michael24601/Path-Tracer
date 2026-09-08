#ifndef PATH_TRACER_PARSER_HPP
#define PATH_TRACER_PARSER_HPP

#include <nlohmann/json.hpp>
#include <unordered_map>
#include "../core/scene.hpp"
#include "../logger.hpp"
#include "../core/instance.hpp"
#include "../bsdf/bsdf.hpp"
#include "../bsdf/diffuseBsdf.hpp"
#include "../light/light.hpp"
#include "../texture/texture.hpp"
#include "../shapes/sphere.hpp"
#include "../shapes/triangle.hpp"
#include "../emission/lambertianEmission.hpp"
#include "../light/areaLight.hpp"
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

        static Scene* parse(std::string filename){

            std::ifstream file("filename");
            json data = json::parse(file);

            if (!file.is_open()){
                LOG_ERROR("File could not be opened");
                exit(1);
            }

            std::unordered_map<std::string, Bsdf*> materials;
            std::unordered_map<std::string, Shape*> shapes;
            // Instances can share an ID
            std::unordered_map<std::string, std::vector<Instance*>> instances;
            std::unordered_map<std::string, Light*> lights;

            // Only one copy needed
            Sphere* sphere = new Sphere();

            if(data.contains("materials")){

                for(int i = 0; i < data["materials"].size(); i++){

                    std::string id = data["materials"][i]["id"];
                    
                    if(data["materials"][i]["type"] == "diffuse"){

                        Texture* texture;
                        if(data["materials"][i]["albedo"]["type"] == "color"){

                            Vector3 color = parseVector3(data["materials"][i]["albedo"]["value"]);
                            // 1 by 1 texture for solid color
                            std::vector<std::vector<Vector3>> texArr{{color}};
                            texture = new Texture(texArr, Texture::BorderMode::CLAMP,
                                Texture::FilterMode::NEAREST);
                        }

                        materials[id] = new DiffuseBsdf(texture);
                    }
                }

            }

            if(data.contains("instances")){
                
                for(int i = 0; i < data["instances"].size(); i++){

                    json& inst = data["instances"][i];
                    std::string id = inst["id"];

                    std::string materialID = inst["material"];
                    if(!materials[materialID]){
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

                    if(inst["shape"] == "sphere"){
                        
                        Transform transform = parseTransform(
                            inst["parameters"]["transform"]
                        );

                        Instance* instance = new Instance(sphere, nullptr,
                            nullptr, materials[materialID], emission, transform);

                        instances[id].push_back(instance);
                    }


                    if(inst["shape"] == "mesh"){
                        
                        Transform transform = parseTransform(
                            inst["parameters"]["transform"]
                        );

                        // We need to loop over every triangle
                        for(int j = 0; j < inst["parameters"]["vertices"].size(); j++){

                            Triangle* triangle = new Triangle(
                                parseVector3(inst["parameters"]["vertices"][j][0]),
                                parseVector3(inst["parameters"]["vertices"][j][1]),
                                parseVector3(inst["parameters"]["vertices"][j][2])
                            );

                            Instance* instance = new Instance(triangle, nullptr,
                                nullptr, materials[materialID], emission, transform);

                            instances[id].push_back(instance);
                        }

                    }
                }
            }


            if(data.contains("lights")){

                for(int i = 0; i < data["lights"].size(); i++){

                    std::string id = data["lights"][i]["id"];
                    
                    if(data["lights"][i]["type"] == "area"){

                        std::string instID = data["lights"][i]["parameters"]["instance"];

                        // Instance should have emission
                        for(int j = 0; j < instances[instID].size(); j++){
                            if(!instances[instID][j]->emission()){
                                LOG_ERROR("Could not find specified emissive instance: " + instID);
                                exit(1);
                            }
                            lights[id] = new AreaLight(instances[instID]);
                        }
                    }
                }  

            }
        }

    };

}

#endif