
#include "../../include/headers/math/vector3.hpp"
#include "../../include/headers/shapes/triangle.hpp"
#include "../../include/headers/core/instance.hpp"
#include "../../include/headers/bsdf/diffuseBsdf.hpp"
#include "../../include/headers/emission/lambertianEmission.hpp"
#include <vector>

namespace pathtracer{

    int width = 100;
    int height = 100;

    std::vector<Triangle*> floor = {
        new Triangle(Vector3(552.8, 0.0, 0.0), Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 559.2)),
        new Triangle(Vector3(552.8, 0.0, 0.0), Vector3(0.0, 0.0, 559.2), Vector3(549.6, 0.0, 559.2))
    };

    std::vector<Triangle*> light = {
        new Triangle(Vector3(343.0, 548.78, 227.0), Vector3(343.0, 548.78, 332.0), Vector3(213.0, 548.78, 332.0)),
        new Triangle(Vector3(343.0, 548.78, 227.0), Vector3(213.0, 548.78, 332.0), Vector3(213.0, 548.78, 227.0))
    };

    std::vector<Triangle*> ceiling = {
        new Triangle(Vector3(556.0, 548.8, 0.0), Vector3(556.0, 548.8, 559.2), Vector3(0.0, 548.8, 559.2)),
        new Triangle(Vector3(556.0, 548.8, 0.0), Vector3(0.0, 548.8, 559.2), Vector3(0.0, 548.8, 0.0))
    };

    std::vector<Triangle*> backWall = {
        new Triangle(Vector3(549.6, 0.0, 559.2), Vector3(0.0, 0.0, 559.2), Vector3(0.0, 548.8, 559.2)),
        new Triangle(Vector3(549.6, 0.0, 559.2), Vector3(0.0, 548.8, 559.2), Vector3(556.0, 548.8, 559.2))
    };

    std::vector<Triangle*> rightWall = {
        new Triangle(Vector3(0.0, 0.0, 559.2), Vector3(0.0, 0.0, 0.0), Vector3(0.0, 548.8, 0.0)),
        new Triangle(Vector3(0.0, 0.0, 559.2), Vector3(0.0, 548.8, 0.0), Vector3(0.0, 548.8, 559.2))
    };

    std::vector<Triangle*> leftWall = {
        new Triangle(Vector3(552.8, 0.0, 0.0), Vector3(549.6, 0.0, 559.2), Vector3(556.0, 548.8, 559.2)),
        new Triangle(Vector3(552.8, 0.0, 0.0), Vector3(556.0, 548.8, 559.2), Vector3(556.0, 548.8, 0.0))
    };

    std::vector<Triangle*> shortBlock = {
        new Triangle(Vector3(130.0, 165.0, 65.0), Vector3(82.0, 165.0, 225.0), Vector3(240.0, 165.0, 272.0)),
        new Triangle(Vector3(130.0, 165.0, 65.0), Vector3(240.0, 165.0, 272.0), Vector3(290.0, 165.0, 114.0)),

        new Triangle(Vector3(290.0, 0.0, 114.0), Vector3(290.0, 165.0, 114.0), Vector3(240.0, 165.0, 272.0)),
        new Triangle(Vector3(290.0, 0.0, 114.0), Vector3(240.0, 165.0, 272.0), Vector3(240.0, 0.0, 272.0)),

        new Triangle(Vector3(130.0, 0.0, 65.0), Vector3(130.0, 165.0, 65.0), Vector3(290.0, 165.0, 114.0)),
        new Triangle(Vector3(130.0, 0.0, 65.0), Vector3(290.0, 165.0, 114.0), Vector3(290.0, 0.0, 114.0)),

        new Triangle(Vector3(82.0, 0.0, 225.0), Vector3(82.0, 165.0, 225.0), Vector3(130.0, 165.0, 65.0)),
        new Triangle(Vector3(82.0, 0.0, 225.0), Vector3(130.0, 165.0, 65.0), Vector3(130.0, 0.0, 65.0)),

        new Triangle(Vector3(240.0, 0.0, 272.0), Vector3(240.0, 165.0, 272.0), Vector3(82.0, 165.0, 225.0)),
        new Triangle(Vector3(240.0, 0.0, 272.0), Vector3(82.0, 165.0, 225.0), Vector3(82.0, 0.0, 225.0))
    };

    std::vector<Triangle*> tallBlock = {
        new Triangle(Vector3(423.0, 330.0, 247.0), Vector3(265.0, 330.0, 296.0), Vector3(314.0, 330.0, 456.0)),
        new Triangle(Vector3(423.0, 330.0, 247.0), Vector3(314.0, 330.0, 456.0), Vector3(472.0, 330.0, 406.0)),

        new Triangle(Vector3(423.0, 0.0, 247.0), Vector3(423.0, 330.0, 247.0), Vector3(472.0, 330.0, 406.0)),
        new Triangle(Vector3(423.0, 0.0, 247.0), Vector3(472.0, 330.0, 406.0), Vector3(472.0, 0.0, 406.0)),

        new Triangle(Vector3(472.0, 0.0, 406.0), Vector3(472.0, 330.0, 406.0), Vector3(314.0, 330.0, 456.0)),
        new Triangle(Vector3(472.0, 0.0, 406.0), Vector3(314.0, 330.0, 456.0), Vector3(314.0, 0.0, 456.0)),

        new Triangle(Vector3(314.0, 0.0, 456.0), Vector3(314.0, 330.0, 456.0), Vector3(265.0, 330.0, 296.0)),
        new Triangle(Vector3(314.0, 0.0, 456.0), Vector3(265.0, 330.0, 296.0), Vector3(265.0, 0.0, 296.0)),

        new Triangle(Vector3(265.0, 0.0, 296.0), Vector3(265.0, 330.0, 296.0), Vector3(423.0, 330.0, 247.0)),
        new Triangle(Vector3(265.0, 0.0, 296.0), Vector3(423.0, 330.0, 247.0), Vector3(423.0, 0.0, 247.0))
    };


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
        new Instance(floor[0], nullptr, nullptr, whiteBsdf, nullptr, Transform::IDENTITY),
        new Instance(floor[1], nullptr, nullptr, whiteBsdf, nullptr, Transform::IDENTITY),

        new Instance(light[0], nullptr, nullptr, lightBsdf, emission, Transform::IDENTITY),
        new Instance(light[1], nullptr, nullptr, lightBsdf, emission, Transform::IDENTITY),

        new Instance(ceiling[0], nullptr, nullptr, whiteBsdf, nullptr, Transform::IDENTITY),
        new Instance(ceiling[1], nullptr, nullptr, whiteBsdf, nullptr, Transform::IDENTITY),

        new Instance(backWall[0], nullptr, nullptr, whiteBsdf, nullptr, Transform::IDENTITY),
        new Instance(backWall[1], nullptr, nullptr, whiteBsdf, nullptr, Transform::IDENTITY),

        new Instance(rightWall[0], nullptr, nullptr, greenBsdf, nullptr, Transform::IDENTITY),
        new Instance(rightWall[1], nullptr, nullptr, greenBsdf, nullptr, Transform::IDENTITY),

        new Instance(leftWall[0], nullptr, nullptr, redBsdf, nullptr, Transform::IDENTITY),
        new Instance(leftWall[1], nullptr, nullptr, redBsdf, nullptr, Transform::IDENTITY),

        new Instance(shortBlock[0], nullptr, nullptr, whiteBsdf, nullptr, Transform::IDENTITY),
        new Instance(shortBlock[1], nullptr, nullptr, whiteBsdf, nullptr, Transform::IDENTITY),
        new Instance(shortBlock[2], nullptr, nullptr, whiteBsdf, nullptr, Transform::IDENTITY),
        new Instance(shortBlock[3], nullptr, nullptr, whiteBsdf, nullptr, Transform::IDENTITY),
        new Instance(shortBlock[4], nullptr, nullptr, whiteBsdf, nullptr, Transform::IDENTITY),
        new Instance(shortBlock[5], nullptr, nullptr, whiteBsdf, nullptr, Transform::IDENTITY),
        new Instance(shortBlock[6], nullptr, nullptr, whiteBsdf, nullptr, Transform::IDENTITY),
        new Instance(shortBlock[7], nullptr, nullptr, whiteBsdf, nullptr, Transform::IDENTITY),
        new Instance(shortBlock[8], nullptr, nullptr, whiteBsdf, nullptr, Transform::IDENTITY),
        new Instance(shortBlock[9], nullptr, nullptr, whiteBsdf, nullptr, Transform::IDENTITY),

        new Instance(tallBlock[0], nullptr, nullptr, whiteBsdf, nullptr, Transform::IDENTITY),
        new Instance(tallBlock[1], nullptr, nullptr, whiteBsdf, nullptr, Transform::IDENTITY),
        new Instance(tallBlock[2], nullptr, nullptr, whiteBsdf, nullptr, Transform::IDENTITY),
        new Instance(tallBlock[3], nullptr, nullptr, whiteBsdf, nullptr, Transform::IDENTITY),
        new Instance(tallBlock[4], nullptr, nullptr, whiteBsdf, nullptr, Transform::IDENTITY),
        new Instance(tallBlock[5], nullptr, nullptr, whiteBsdf, nullptr, Transform::IDENTITY),
        new Instance(tallBlock[6], nullptr, nullptr, whiteBsdf, nullptr, Transform::IDENTITY),
        new Instance(tallBlock[7], nullptr, nullptr, whiteBsdf, nullptr, Transform::IDENTITY),
        new Instance(tallBlock[8], nullptr, nullptr, whiteBsdf, nullptr, Transform::IDENTITY),
        new Instance(tallBlock[9], nullptr, nullptr, whiteBsdf, nullptr, Transform::IDENTITY)
    };

}