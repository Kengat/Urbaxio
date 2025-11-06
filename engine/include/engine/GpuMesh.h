#pragma once

#include <glad/glad.h>
#include <cstddef> // for size_t

namespace Urbaxio::Engine {

    // A plain data structure to hold OpenGL handles for a mesh on the GPU.
    struct GpuMesh {
        GLuint vao = 0;
        GLuint vbo_vertices = 0;
        GLuint vbo_normals = 0;
        GLuint vbo_uvs = 0;
        GLuint ebo = 0;
        GLsizei index_count = 0;

        // An additional VBO for instance-specific data (e.g., model matrices)
        GLuint vbo_instances = 0; 
    };

} // namespace Urbaxio::Engine


