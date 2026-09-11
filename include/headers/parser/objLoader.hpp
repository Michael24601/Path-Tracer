
#ifndef PATH_TRACER_OBJ_LOADER_H
#define PATH_TRACER_OBJ_LOADER_H

// Needed for tiny_obj_loader
#define TINYOBJLOADER_IMPLEMENTATION

#include <obj-loader/tiny_obj_loader.h>
#include "../shapes/mesh.hpp"

namespace pathtracer{

    class ObjLoader {

    private:


        static Vector3 parseObjVector3(
            const std::vector<float>& data,
            int index
        ) {
            return Vector3(
                data[3 * index],
                data[3 * index + 1],
                data[3 * index + 2]
            );
        }

        static Vector2 parseObjVector2(
            const std::vector<float>& data,
            int index
        ) {
            return Vector2(
                data[2 * index],
                data[2 * index + 1]
            );
        }


    public:

        // Given an OBJ file, loads a mesh
        static Mesh* loadMesh(std::string filename) {

            tinyobj::attrib_t attrib;
            std::vector<tinyobj::shape_t> shapes;
            std::vector<tinyobj::material_t> materials;

            std::string warn;
            std::string err;


            if (!tinyobj::LoadObj(
                    &attrib,
                    &shapes,
                    &materials,
                    &warn,
                    &err,
                    filename.c_str())){

                throw std::runtime_error(err);
            }

            std::vector<Triangle> data;

            for (const auto& shape : shapes) {

                const auto& indices = shape.mesh.indices;

                for (size_t i = 0; i < indices.size(); i += 3) {

                    const auto& i0 = indices[i];
                    const auto& i1 = indices[i + 1];
                    const auto& i2 = indices[i + 2];

                    Vector3 v0 = parseObjVector3(attrib.vertices, i0.vertex_index);
                    Vector3 v1 = parseObjVector3(attrib.vertices, i1.vertex_index);
                    Vector3 v2 = parseObjVector3(attrib.vertices, i2.vertex_index);

                    Vector3 n0;
                    Vector3 n1;
                    Vector3 n2;

                    Vector2 uv0;
                    Vector2 uv1;
                    Vector2 uv2;

                    bool hasNormals = i0.normal_index >= 0;
                    bool hasUVs = i0.texcoord_index >= 0;

                    if (hasNormals) {
                        n0 = parseObjVector3(attrib.normals, i0.normal_index);
                        n1 = parseObjVector3(attrib.normals, i1.normal_index);
                        n2 = parseObjVector3(attrib.normals, i2.normal_index);
                    }

                    if (hasUVs) {
                        uv0 = parseObjVector2(attrib.texcoords, i0.texcoord_index);
                        uv1 = parseObjVector2(attrib.texcoords, i1.texcoord_index);
                        uv2 = parseObjVector2(attrib.texcoords, i2.texcoord_index);
                    }

                    if (hasNormals && hasUVs)
                        data.push_back(Triangle(v0, v1, v2, n0, n1, n2, uv0, uv1, uv2));
                    else if (hasNormals)
                        data.push_back(Triangle(v0, v1, v2, n0, n1, n2));
                    else if (hasUVs)
                        data.push_back(Triangle(v0, v1, v2, uv0, uv1, uv2));
                    else
                        data.push_back(Triangle(v0, v1, v2));
                }
            }

            return new Mesh(data);
        }
    };
}

#endif