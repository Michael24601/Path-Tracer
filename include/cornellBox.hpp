
#include "headers/math/vector3.hpp"
#include "headers/shapes/triangle.hpp"
#include "headers/core/instance.hpp"
#include "headers/bsdf/diffuseBsdf.hpp"
#include "headers/emission/lambertianEmission.hpp"
#include "headers/shapes/mesh.hpp"
#include <vector>

namespace pathtracer{

    int width = 100;
    int height = 100;

    std::vector<Triangle> floor = {
        Triangle(Vector3(552.8, 0.0, 0.0), Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 559.2)),
        Triangle(Vector3(552.8, 0.0, 0.0), Vector3(0.0, 0.0, 559.2), Vector3(549.6, 0.0, 559.2))
    };
    Mesh* mesh1 = new Mesh(floor);

    std::vector<Triangle> light = {
        Triangle(Vector3(343.0, 548.78, 227.0), Vector3(343.0, 548.78, 332.0), Vector3(213.0, 548.78, 332.0)),
        Triangle(Vector3(343.0, 548.78, 227.0), Vector3(213.0, 548.78, 332.0), Vector3(213.0, 548.78, 227.0))
    };
    Mesh* mesh2 = new Mesh(light);

    std::vector<Triangle> ceiling = {
        Triangle(Vector3(556.0, 548.8, 0.0), Vector3(556.0, 548.8, 559.2), Vector3(0.0, 548.8, 559.2)),
        Triangle(Vector3(556.0, 548.8, 0.0), Vector3(0.0, 548.8, 559.2), Vector3(0.0, 548.8, 0.0))
    };
    Mesh* mesh3 = new Mesh(ceiling);

    std::vector<Triangle> backWall = {
        Triangle(Vector3(549.6, 0.0, 559.2), Vector3(0.0, 0.0, 559.2), Vector3(0.0, 548.8, 559.2)),
        Triangle(Vector3(549.6, 0.0, 559.2), Vector3(0.0, 548.8, 559.2), Vector3(556.0, 548.8, 559.2))
    };
    Mesh* mesh4 = new Mesh(backWall);

    std::vector<Triangle> rightWall = {
        Triangle(Vector3(0.0, 0.0, 559.2), Vector3(0.0, 0.0, 0.0), Vector3(0.0, 548.8, 0.0)),
        Triangle(Vector3(0.0, 0.0, 559.2), Vector3(0.0, 548.8, 0.0), Vector3(0.0, 548.8, 559.2))
    };
    Mesh* mesh5 = new Mesh(rightWall);

    std::vector<Triangle> leftWall = {
        Triangle(Vector3(552.8, 0.0, 0.0), Vector3(549.6, 0.0, 559.2), Vector3(556.0, 548.8, 559.2)),
        Triangle(Vector3(552.8, 0.0, 0.0), Vector3(556.0, 548.8, 559.2), Vector3(556.0, 548.8, 0.0))
    };
    Mesh* mesh6 = new Mesh(leftWall);

    std::vector<Triangle> shortBlock = {
        Triangle(Vector3(130.0, 165.0, 65.0), Vector3(82.0, 165.0, 225.0), Vector3(240.0, 165.0, 272.0)),
        Triangle(Vector3(130.0, 165.0, 65.0), Vector3(240.0, 165.0, 272.0), Vector3(290.0, 165.0, 114.0)),

        Triangle(Vector3(290.0, 0.0, 114.0), Vector3(290.0, 165.0, 114.0), Vector3(240.0, 165.0, 272.0)),
        Triangle(Vector3(290.0, 0.0, 114.0), Vector3(240.0, 165.0, 272.0), Vector3(240.0, 0.0, 272.0)),

        Triangle(Vector3(130.0, 0.0, 65.0), Vector3(130.0, 165.0, 65.0), Vector3(290.0, 165.0, 114.0)),
        Triangle(Vector3(130.0, 0.0, 65.0), Vector3(290.0, 165.0, 114.0), Vector3(290.0, 0.0, 114.0)),

        Triangle(Vector3(82.0, 0.0, 225.0), Vector3(82.0, 165.0, 225.0), Vector3(130.0, 165.0, 65.0)),
        Triangle(Vector3(82.0, 0.0, 225.0), Vector3(130.0, 165.0, 65.0), Vector3(130.0, 0.0, 65.0)),

        Triangle(Vector3(240.0, 0.0, 272.0), Vector3(240.0, 165.0, 272.0), Vector3(82.0, 165.0, 225.0)),
        Triangle(Vector3(240.0, 0.0, 272.0), Vector3(82.0, 165.0, 225.0), Vector3(82.0, 0.0, 225.0))
    };
    Mesh* mesh7 = new Mesh(shortBlock);

    std::vector<Triangle> tallBlock = {
        Triangle(Vector3(423.0, 330.0, 247.0), Vector3(265.0, 330.0, 296.0), Vector3(314.0, 330.0, 456.0)),
        Triangle(Vector3(423.0, 330.0, 247.0), Vector3(314.0, 330.0, 456.0), Vector3(472.0, 330.0, 406.0)),

        Triangle(Vector3(423.0, 0.0, 247.0), Vector3(423.0, 330.0, 247.0), Vector3(472.0, 330.0, 406.0)),
        Triangle(Vector3(423.0, 0.0, 247.0), Vector3(472.0, 330.0, 406.0), Vector3(472.0, 0.0, 406.0)),

        Triangle(Vector3(472.0, 0.0, 406.0), Vector3(472.0, 330.0, 406.0), Vector3(314.0, 330.0, 456.0)),
        Triangle(Vector3(472.0, 0.0, 406.0), Vector3(314.0, 330.0, 456.0), Vector3(314.0, 0.0, 456.0)),

        Triangle(Vector3(314.0, 0.0, 456.0), Vector3(314.0, 330.0, 456.0), Vector3(265.0, 330.0, 296.0)),
        Triangle(Vector3(314.0, 0.0, 456.0), Vector3(265.0, 330.0, 296.0), Vector3(265.0, 0.0, 296.0)),

        Triangle(Vector3(265.0, 0.0, 296.0), Vector3(265.0, 330.0, 296.0), Vector3(423.0, 330.0, 247.0)),
        Triangle(Vector3(265.0, 0.0, 296.0), Vector3(423.0, 330.0, 247.0), Vector3(423.0, 0.0, 247.0))
    };
    Mesh* mesh8 = new Mesh(tallBlock);


    std::vector<std::vector<Vector3>> whiteAlbedo(height,
    std::vector<Vector3>(width, Vector3(0.725, 0.71, 0.68)));

    Texture* whiteAlbedoTexture = new Texture(whiteAlbedo,
        Texture::BorderMode::CLAMP,
        Texture::FilterMode::NEAREST
    );

    std::vector<std::vector<Vector3>> redAlbedo(height,
        std::vector<Vector3>(width, Vector3(0.63, 0.065, 0.05)));

    Texture* redAlbedoTexture = new Texture(redAlbedo,
        Texture::BorderMode::CLAMP,
        Texture::FilterMode::NEAREST
    );

    std::vector<std::vector<Vector3>> greenAlbedo(height,
        std::vector<Vector3>(width, Vector3(0.14, 0.45, 0.091)));

    Texture* greenAlbedoTexture = new Texture(greenAlbedo,
        Texture::BorderMode::CLAMP,
        Texture::FilterMode::NEAREST
    );

    std::vector<std::vector<Vector3>> lightAlbedo(height,
        std::vector<Vector3>(width, Vector3(1.0, 1.0, 1.0)));

    Texture* lightAlbedoTexture = new Texture(lightAlbedo,
        Texture::BorderMode::CLAMP,
        Texture::FilterMode::NEAREST
    );


    DiffuseBsdf* whiteBsdf = new DiffuseBsdf(whiteAlbedoTexture);
    DiffuseBsdf* redBsdf = new DiffuseBsdf(redAlbedoTexture);
    DiffuseBsdf* greenBsdf = new DiffuseBsdf(greenAlbedoTexture);
    DiffuseBsdf* lightBsdf = new DiffuseBsdf(lightAlbedoTexture);

    LambertianEmission* emission = new LambertianEmission(Vector3(17.0, 12.0, 4.0));

    std::vector<Instance*> instances = {
        new Instance(mesh1, nullptr, nullptr, whiteBsdf, nullptr, Transform::IDENTITY),

        new Instance(mesh2, nullptr, nullptr, lightBsdf, emission, Transform::IDENTITY),

        new Instance(mesh3, nullptr, nullptr, whiteBsdf, nullptr, Transform::IDENTITY),

        new Instance(mesh4, nullptr, nullptr, whiteBsdf, nullptr, Transform::IDENTITY),

        new Instance(mesh5, nullptr, nullptr, greenBsdf, nullptr, Transform::IDENTITY),

        new Instance(mesh6, nullptr, nullptr, redBsdf, nullptr, Transform::IDENTITY),

        new Instance(mesh7, nullptr, nullptr, whiteBsdf, nullptr, Transform::IDENTITY),

        new Instance(mesh8, nullptr, nullptr, whiteBsdf, nullptr, Transform::IDENTITY),
    };


    // Adds them as lights
    AreaLight* light_l = new AreaLight(instances[1]);
    std::vector<Light*> lights {light_l};

}