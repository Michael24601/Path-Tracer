
#include "../include/headers/renderer/renderer.hpp"
#include "../include/libs/stb_image_write.h"
#include "../include/headers/camera/perspectiveCamera.hpp"
#include "../include/headers/shapes/sphere.hpp"
#include "../include/headers/bsdf/diffuseBsdf.hpp"
#include "../include/headers/emission/lambertianEmission.hpp"
#include "../include/headers/renderer/renderer.hpp"
#include "../include/headers/integrator/aovIntegrator.hpp"
#include "../include/headers/integrator/pathTracer.hpp"

using namespace pathtracer;

void savePNG(const std::vector<std::vector<Vector3>>& image,
             const std::string& filename) {

    int height = static_cast<int>(image.size());
    if (height == 0) return;

    int width = static_cast<int>(image[0].size());
    if (width == 0) return;

    std::vector<unsigned char> pixels(width * height * 3);

    auto gammaCorrect = [](double v) {
        v = std::clamp(v, 0.0, 1.0);
        return std::pow(v, 1.0 / 2.2);
    };

    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {

            const Vector3& c = image[j][i];

            int index = (j * width + i) * 3;

            pixels[index + 0] = static_cast<unsigned char>(
                gammaCorrect(c.x()) * 255.0 + 0.5);

            pixels[index + 1] = static_cast<unsigned char>(
                gammaCorrect(c.y()) * 255.0 + 0.5);

            pixels[index + 2] = static_cast<unsigned char>(
                gammaCorrect(c.z()) * 255.0 + 0.5);
        }
    }

    stbi_write_png(
        filename.c_str(),
        width,
        height,
        3,
        pixels.data(),
        width * 3
    );
}

int main(){

    real width = 1.0;
    real height = 0.7;

    int texPixelsH = 100;
    int texPixelsW = 150;

    std::vector<std::vector<Vector3>> albedo(100, 
        std::vector<Vector3>(150, Vector3(1.0, 0.0, 0.0)));

    Transform camTransform = Camera::lookAt(Vector3(-3.0, 0, 0), 
        Vector3::ORIGIN, Vector3(0, 0, 1));
    real focalLength = 0.7;

    PerspectiveCamera* camera = new PerspectiveCamera(width, 
        height, camTransform, focalLength);

    Sphere* sphere = new Sphere();
    Texture* albedoTexture = new Texture(albedo, 
        Texture::BorderMode::CLAMP, 
        Texture::FilterMode::NEAREST
    );

    DiffuseBsdf* diffuseBsdf = new DiffuseBsdf(albedoTexture);
    LambertianEmission* emission = new LambertianEmission(Vector3(1.0));

    Instance* sphereInst = new Instance(sphere, nullptr, 
        nullptr, diffuseBsdf, nullptr, Transform::IDENTITY);

    Transform lightTransform(Matrix3(Vector3(0.5, 0.0, 0.0), 
        Vector3(0.0, 0.5, 0.0), 
        Vector3(0.0, 0.0, 0.5)),
        Vector3(-1.0, 1.0, -1.0));

    Instance* lightInst = new Instance(sphere, nullptr, 
        nullptr, diffuseBsdf, emission, lightTransform);
    std::vector<Instance*> instances {lightInst, sphereInst};

    Scene* scene = new Scene(instances, std::vector<Light*>{});

    AovIntegrator* integrator = new AovIntegrator(
        AovIntegrator::RenderVariable::NORMAL);
    PathTracer* pathtracer = new PathTracer(3, 50);
    
    Renderer renderer(500, 350, camera, scene, pathtracer);
    auto image = renderer.render();

    savePNG(image, "image.png");

    return 0;
}