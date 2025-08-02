#ifndef URBAXIO_SCENE_OBJECT_H
#define URBAXIO_SCENE_OBJECT_H

#include <cstdint>
#include <string>
#include <memory>
#include <vector>
#include <set> // <-- ADDED for boundaryLineIDs
#include <unordered_map> // <-- ADDED
#include <cad_kernel/cad_kernel.h>
#include <cad_kernel/MeshBuffers.h>
#include <glad/glad.h> //     GLuint

class TopoDS_Shape;

namespace Urbaxio::Engine {

    class SceneObject {
    public:
        SceneObject(uint64_t id, std::string name);
        ~SceneObject();

        SceneObject(const SceneObject&) = delete;
        SceneObject& operator=(const SceneObject&) = delete;
        SceneObject(SceneObject&&) noexcept;
        SceneObject& operator=(SceneObject&&) noexcept;

        uint64_t get_id() const;
        const std::string& get_name() const;
        void set_name(const std::string& name);

        //           BRep
        void set_shape(Urbaxio::CadKernel::OCCT_ShapeUniquePtr shape);
        const TopoDS_Shape* get_shape() const;
        bool has_shape() const;

        //                (CPU)
        void set_mesh_buffers(Urbaxio::CadKernel::MeshBuffers buffers);
        const Urbaxio::CadKernel::MeshBuffers& get_mesh_buffers() const;
        bool has_mesh() const;

        //                       (GPU)
        GLuint vao = 0;
        GLuint vbo_vertices = 0;
        GLuint vbo_normals = 0;
        GLuint ebo = 0;
        GLsizei index_count = 0;
        
        // --- NEW: Link to boundary lines ---
        std::set<uint64_t> boundaryLineIDs;

        // --- NEW: Adjacency map for mesh vertices ("sticky geometry" support) ---
        // Maps a vertex index to a set of indices of its direct neighbors.
        std::unordered_map<unsigned int, std::set<unsigned int>> meshAdjacency;

    private:
        uint64_t id_;
        std::string name_;
        Urbaxio::CadKernel::OCCT_ShapeUniquePtr shape_ = nullptr;
        Urbaxio::CadKernel::MeshBuffers mesh_buffers_;
    };

} // namespace Urbaxio::Engine

#endif // URBAXIO_SCENE_OBJECT_H