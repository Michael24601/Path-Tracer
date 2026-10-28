
#ifndef PATH_TRACER_OBJ_LOADER_HPP
#define PATH_TRACER_OBJ_LOADER_HPP

#include <string>

namespace pathtracer{

    class Mesh;

    namespace ObjLoader{

        // Given an OBJ file, loads a mesh
        Mesh* loadMesh(std::string filename);

    }

}

#endif