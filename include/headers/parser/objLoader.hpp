
#ifndef PATH_TRACER_OBJ_LOADER_HPP
#define PATH_TRACER_OBJ_LOADER_HPP

#include <string>
#include <memory>

namespace pathtracer{

    class Mesh;

    namespace ObjLoader{

        // Given an OBJ file, loads a mesh
        std::unique_ptr<Mesh> loadMesh(std::string filename);

    }

}

#endif