
#include "parser/parser.hpp"
#include <fstream>
#include <unordered_map>
#include <nlohmann/json.hpp>
#include "core/instance.hpp"
#include "core/scene.hpp"
#include "core/transform.hpp"
#include "bsdf/bsdf.hpp"
#include "bsdf/diffuseBsdf.hpp"
#include "bsdf/principledBsdf.hpp"
#include "bsdf/mirrorBsdf.hpp"
#include "bsdf/dielectricBsdf.hpp"
#include "camera/perspectiveCamera.hpp"
#include "emission/lambertianEmission.hpp"
#include "image/imageIo.hpp"
#include "integrator/aovIntegrator.hpp"
#include "integrator/pathTracer.hpp"
#include "integrator/pathTracerNee.hpp"
#include "integrator/pathTracerNeeMis.hpp"
#include "integrator/pathTracerGuided.hpp"
#include "integrator/pathTracerNeeGuided.hpp"
#include "integrator/pathTracerSms.hpp"
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
#include "shapes/mesh.hpp"
#include "shapes/sphere.hpp"
#include "shapes/triangle.hpp"
#include "texture/texture.hpp"
#include "texture/color.hpp"
#include "parser/objLoader.hpp"
#include <nlohmann/json.hpp>


namespace pathtracer{


    using json = nlohmann::json;


    Vector3 parseVector3(const json& value) {
        return Vector3(
            value[0],
            value[1],
            value[2]
        );
    }


    Vector2 parseVector2(const json& value) {
        return Vector2(
            value[0],
            value[1]
        );
    }
    

    Vector2i parseVector2i(const json& value) {
        return Vector2i(
            value[0],
            value[1]
        );
    }


    // Expects json of a texture
    Texture* parseTexture(const json& texture) {

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
            else if(value["filter-mode"] == "bilinear"){
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

    
    Transform parseTransform(const json& value) {
        // We assume the transform matrix is defined row by row,
        // But matrix is defined column by column
        return Transform(
            Matrix3(
                Vector3(value[0], value[4], value[8]),
                Vector3(value[1], value[5], value[9]),
                Vector3(value[2], value[6], value[10])
            ),
            Vector3(value[3], value[7], value[11])
        );
    }



    // Parser member function
    Renderer* Parser::parse(const std::string& filename) {

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

                Vector3 eye = parseVector3(
                    data["camera"]["parameters"]["look-at"]["eye"]);

                Vector3 target = parseVector3(
                    data["camera"]["parameters"]["look-at"]["target"]);

                Vector3 up = parseVector3(
                    data["camera"]["parameters"]["look-at"]["up"]);

                Vector2 dim = parseVector2(
                    data["camera"]["parameters"]["sensor-dimension"]);

                real focalLength =
                    data["camera"]["parameters"]["focal-length"];

                camera = new PerspectiveCamera(
                    dim.x(),
                    dim.y(),
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

                    Texture* texture = parseTexture(
                        data["materials"][i]["parameters"]["albedo"]);

                    materials[id] = new DiffuseBsdf(texture);
                }
                else if(data["materials"][i]["type"] == "mirror"){

                    real reflectance =
                        data["materials"][i]["parameters"]["reflectance"];

                    materials[id] = new MirrorBsdf(reflectance);
                }
                else if(data["materials"][i]["type"] == "dielectric"){

                    Texture* ior = parseTexture(
                        data["materials"][i]["parameters"]["ior"]);

                    Texture* reflectance = parseTexture(
                        data["materials"][i]["parameters"]["reflectance"]);

                    Texture* transmittance = parseTexture(
                        data["materials"][i]["parameters"]["transmittance"]);

                    materials[id] =
                        new DielectricBsdf(ior, reflectance, transmittance);
                }
                else if(data["materials"][i]["type"] == "principled"){

                    Texture* baseColor = nullptr;

                    // Default values
                    Texture* roughness = new Color(Vector3(0.0));
                    Texture* metallic = new Color(Vector3(0.0));
                    Texture* specular = new Color(Vector3(0.0));

                    baseColor = parseTexture(
                        data["materials"][i]["parameters"]["base-color"]);

                    if(data["materials"][i]["parameters"].contains("roughness")){
                        roughness = parseTexture(
                            data["materials"][i]["parameters"]["roughness"]);
                    }

                    if(data["materials"][i]["parameters"].contains("metallic")){
                        metallic = parseTexture(
                            data["materials"][i]["parameters"]["metallic"]);
                    }

                    if(data["materials"][i]["parameters"].contains("specular")){
                        specular = parseTexture(
                            data["materials"][i]["parameters"]["specular"]);
                    }

                    materials[id] =
                        new PrincipledBsdf(
                            baseColor,
                            roughness,
                            metallic,
                            specular);
                }
            }
        }

        if(data.contains("instances")){

            for(int i = 0; i < data["instances"].size(); i++){

                json& inst = data["instances"][i];
                std::string id = inst["id"];

                std::string materialID = inst["material"];

                if(materials.find(materialID) == materials.end()){
                    LOG_ERROR(
                        "Could not find specified material: " + materialID);

                    exit(1);
                }

                Emission* emission = nullptr;

                if(inst.contains("emission")){
                    if(inst["emission"]["type"] == "lambertian"){
                        emission = new LambertianEmission(
                            parseVector3(
                                inst["emission"]["parameters"]["power"]));
                    }
                }

                Transform transform = Transform::IDENTITY;

                if(inst["parameters"].contains("transform")){
                    transform =
                        parseTransform(inst["parameters"]["transform"]);
                }


                bool isCausticReceiver = false;
                if(inst["parameters"].contains("is-caustic-receiver")){
                    isCausticReceiver = inst["parameters"]["is-caustic-receiver"];
                }

                Texture* normalMap = nullptr;

                if(inst["parameters"].contains("normal-map")){
                    normalMap =
                        parseTexture(inst["parameters"]["normal-map"]);
                }

                Texture* alphaMask = nullptr;

                if(inst["parameters"].contains("alpha-mask")){
                    alphaMask =
                        parseTexture(inst["parameters"]["alpha-mask"]);
                }

                Instance* instance;
                if(inst["type"] == "sphere"){

                    instance = new Instance(
                        sphere,
                        nullptr,
                        nullptr,
                        materials[materialID],
                        emission,
                        transform);
                }
                else if (inst["type"] == "mesh"){

                    Mesh* mesh;

                    if(inst["parameters"]["vertex-data"]["format"] == "raw-array"){

                        std::vector<Triangle> triangles;

                        triangles.reserve(
                            inst["parameters"]["vertex-data"]
                                ["parameters"]["vertices"].size());

                        // We need to loop over every triangle
                        for(int j = 0;
                            j < inst["parameters"]["vertex-data"]
                                ["parameters"]["vertices"].size();
                            j++){

                            triangles.emplace_back(Triangle(
                                parseVector3(
                                    inst["parameters"]["vertex-data"]
                                        ["parameters"]["vertices"][j][0]),

                                parseVector3(
                                    inst["parameters"]["vertex-data"]
                                        ["parameters"]["vertices"][j][1]),

                                parseVector3(
                                    inst["parameters"]["vertex-data"]
                                        ["parameters"]["vertices"][j][2])
                            ));
                        }

                        mesh = new Mesh(triangles);
                    }
                    else if(inst["parameters"]["vertex-data"]["format"] == "obj"){

                        std::string filename =
                            inst["parameters"]["vertex-data"]
                                ["parameters"]["file-name"];

                        mesh = ObjLoader::loadMesh(filename);
                    }

                    instance = new Instance(
                        mesh,
                        alphaMask,
                        normalMap,
                        materials[materialID],
                        emission,
                        transform);
                }
                else{
                    LOG_ERROR("Instance does not have a valid shape.");
                    exit(1);
                }

                instance->setCausticReceiver(isCausticReceiver);
                instances[id] = instance;
            }
        }

        if(data.contains("lights")){

            for(int i = 0; i < data["lights"].size(); i++){

                std::string id = data["lights"][i]["id"];

                if(data["lights"][i]["type"] == "area"){

                    std::string instID =
                        data["lights"][i]["parameters"]["instance"];

                    // Instance should have emission
                    if(instances.find(instID) == instances.end()){
                        LOG_ERROR(
                            "Could not find specified emissive instance: "
                            + instID);

                        exit(1);
                    }


                    if(!instances[instID]->emission()){
                        LOG_ERROR("Instance is not emissive: " + instID);
                        exit(1);
                    }

                    lights[id] =
                        new AreaLight(instances[instID]);
                }
                else if(data["lights"][i]["type"] == "point"){

                    Vector3 power = parseVector3(
                        data["lights"][i]["parameters"]["power"]);

                    Vector3 pos = parseVector3(
                        data["lights"][i]["parameters"]["position"]);

                    lights[id] = new PointLight(pos, power);
                }
                else if(data["lights"][i]["type"] == "directional"){

                    Vector3 power = parseVector3(
                        data["lights"][i]["parameters"]["power"]);

                    Vector3 dir = parseVector3(
                        data["lights"][i]["parameters"]["direction"]);

                    lights[id] = new DirectionalLight(dir, power);
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

            Vector2i resolution =
                parseVector2i(
                    data["renderer"]["parameters"]["pixel-resolution"]);

            if(data["renderer"]["integrator"]["type"] == "aov-integrator"){

                std::string var =
                    data["renderer"]["integrator"]["parameters"]
                        ["render-variable"];

                if(var == "normal"){
                    integrator =
                        new AovIntegrator(
                            AovIntegrator::RenderVariable::NORMAL);
                }
                else if(var == "dpdu"){
                    integrator =
                        new AovIntegrator(
                            AovIntegrator::RenderVariable::DPDU);
                }
                else if(var == "dpdv"){
                    integrator =
                        new AovIntegrator(
                            AovIntegrator::RenderVariable::DPDV);
                }
                else if(var == "dndu"){
                    integrator =
                        new AovIntegrator(
                            AovIntegrator::RenderVariable::DNDU);
                }
                else if(var == "dndv"){
                    integrator =
                        new AovIntegrator(
                            AovIntegrator::RenderVariable::DNDV);
                }
                else if(var == "s"){
                    integrator =
                        new AovIntegrator(
                            AovIntegrator::RenderVariable::S);
                }
                else if(var == "dsdu"){
                    integrator =
                        new AovIntegrator(
                            AovIntegrator::RenderVariable::DSDU);
                }
                else if(var == "dsdv"){
                    integrator =
                        new AovIntegrator(
                            AovIntegrator::RenderVariable::DSDV);
                }
                else if(var == "dtdu"){
                    integrator =
                        new AovIntegrator(
                            AovIntegrator::RenderVariable::DTDU);
                }
                else if(var == "dtdv"){
                    integrator =
                        new AovIntegrator(
                            AovIntegrator::RenderVariable::DTDV);
                }
            }
            else if(data["renderer"]["integrator"]["type"] == "path-tracer"){

                int maxDepth =
                    data["renderer"]["integrator"]["parameters"]["max-depth"];

                integrator = new PathTracer(maxDepth);
            }
            else if(data["renderer"]["integrator"]["type"] == "path-tracer-nee"){

                int maxDepth =
                    data["renderer"]["integrator"]["parameters"]["max-depth"];

                integrator = new PathTracerNee(maxDepth);
            }
            else if(data["renderer"]["integrator"]["type"] == "path-tracer-nee-mis"){

                int maxDepth =
                    data["renderer"]["integrator"]["parameters"]["max-depth"];

                integrator = new PathTracerNeeMis(maxDepth);
            }
            else if(data["renderer"]["integrator"]["type"] == "path-tracer-guided"){

                int maxDepth =
                    data["renderer"]["integrator"]["parameters"]["max-depth"];

                real alpha =
                    data["renderer"]["integrator"]["parameters"]["mis-alpha"];

                integrator =
                    new PathTracerGuided(maxDepth, alpha);
            }
            else if(data["renderer"]["integrator"]["type"] == "path-tracer-nee-guided"){

                int maxDepth =
                    data["renderer"]["integrator"]["parameters"]["max-depth"];

                real alpha =
                    data["renderer"]["integrator"]["parameters"]["mis-alpha"];

                integrator =
                    new PathTracerNeeGuided(maxDepth, alpha);
            }
            else if(data["renderer"]["integrator"]["type"] == "path-tracer-sms"){

                int maxDepth =
                    data["renderer"]["integrator"]["parameters"]["max-depth"];

                integrator =
                    new PathTracerSms(maxDepth, scene);
            }

            if(data["renderer"]["type"] == "path-tracer-renderer"){

                int sampleCount =
                    data["renderer"]["parameters"]["sample-count"];

                renderer = new PathTracerRenderer(
                    resolution.x(),
                    resolution.y(),
                    camera,
                    scene,
                    integrator,
                    sampleCount);
            }
            else if(data["renderer"]["type"] == "renderer"){

                renderer = new Renderer(
                    resolution.x(),
                    resolution.y(),
                    camera,
                    scene,
                    integrator);
            }
            else if(data["renderer"]["type"] == "path-guiding-renderer"){

                int renderSamples =
                    data["renderer"]["parameters"]["render-sample-count"];

                int firstIterationSamples =
                    data["renderer"]["parameters"]
                        ["first-iteration-sample-count"];

                int iterationCount =
                    data["renderer"]["parameters"]["iteration-count"];

                int c =
                    data["renderer"]["parameters"]
                        ["spatial-subdivision-c"];

                renderer = new PathGuidingRenderer(
                    resolution.x(),
                    resolution.y(),
                    camera,
                    scene,
                    integrator,
                    firstIterationSamples,
                    iterationCount,
                    renderSamples,
                    c);
            }
        }
        else{
            LOG_ERROR("No renderer in: " + filename);
            exit(1);
        }

        return renderer;
    }

}