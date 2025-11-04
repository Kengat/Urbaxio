#ifndef URBAXIO_SCENE_OBJECT_H
#define URBAXIO_SCENE_OBJECT_H

#include <cstdint>
#include <string>
#include <memory>
#include <vector>
#include <set> // <-- ADDED for boundaryLineIDs
#include <unordered_map> // <-- ADDED
// #include <map> // Больше не нужен здесь
#include <cad_kernel/cad_kernel.h>
#include <cad_kernel/MeshBuffers.h>
#include <glad/glad.h>
#include <glm/glm.hpp>      // <-- ДОБАВИТЬ ЭТОТ ИНКЛУД
#include "engine/scene.h"   // <-- ДОБАВИТЬ ЭТОТ ИНКЛУД ДЛЯ Vec3Comparator //     GLuint
#include "engine/MeshGroup.h" // <-- ADD THIS INCLUDE

class TopoDS_Shape;
class TopoDS_Vertex;

// Forward-declaration for comparator
// Urbaxio::Vec3Comparator теперь известен из scene.h
// namespace Urbaxio { struct Vec3Comparator; }

namespace Urbaxio::Engine {

    // Forward-declaration for the PIMPL implementation struct
    struct LocationToVertexMapImpl;

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

        // --- NEW: Exportability flag ---
        void setExportable(bool exportable);
        bool isExportable() const;

        //           BRep
        void set_shape(Urbaxio::CadKernel::OCCT_ShapeUniquePtr shape);
        const TopoDS_Shape* get_shape() const;
        bool has_shape() const;

        //                (CPU)
        void set_mesh_buffers(Urbaxio::CadKernel::MeshBuffers buffers);
        const Urbaxio::CadKernel::MeshBuffers& get_mesh_buffers() const;
        bool has_mesh() const;

        // --- NEW: Mesh Groups for Materials ---
        std::vector<MeshGroup> meshGroups;

        //                       (GPU)
        GLuint vao = 0;
        GLuint vbo_vertices = 0;
        GLuint vbo_normals = 0;
        GLuint vbo_uvs = 0; // <-- ADD THIS
        GLuint ebo = 0;
        GLsizei index_count = 0;
        
        // --- NEW: Link to boundary lines ---
        std::set<uint64_t> boundaryLineIDs;

        // --- NEW: Adjacency map for mesh vertices ("sticky geometry" support) ---
        // Maps a vertex index to a set of indices of its direct neighbors.
        std::unordered_map<unsigned int, std::set<unsigned int>> meshAdjacency;

        // --- PIMPL for the location map ---
        void buildLocationToVertexMap(); // Новый метод
        const TopoDS_Vertex* findVertexAtLocation(const glm::vec3& location) const; // Новый метод

    private:
        uint64_t id_;
        std::string name_;
        Urbaxio::CadKernel::OCCT_ShapeUniquePtr shape_ = nullptr;
        bool isExportable_ = true; // By default, all objects are exportable
        Urbaxio::CadKernel::MeshBuffers mesh_buffers_;

        // Указатель на реализацию, скрывающий std::map
        std::unique_ptr<LocationToVertexMapImpl> locationToVertexMapPimpl_;
    };

} // namespace Urbaxio::Engine

#endif // URBAXIO_SCENE_OBJECT_H