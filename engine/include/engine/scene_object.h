#ifndef URBAXIO_SCENE_OBJECT_H
#define URBAXIO_SCENE_OBJECT_H

#include <cstdint>
#include <string>
#include <memory>
#include <vector>
#include <set> // <-- ADDED for boundaryLineIDs
#include <unordered_map> // <-- ADDED
#include <map>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include "engine/scene.h"   // For Vec3Comparator
#include "engine/MeshGroup.h"
#include "engine/geometry/IGeometry.h" // <-- NEW
#include <cad_kernel/MeshBuffers.h>    // <-- Keep for cache member

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

        // --- NEW: Object Transform ---
        void setTransform(const glm::mat4& newTransform);
        const glm::mat4& getTransform() const;

        // --- NEW: Exportability flag ---
        void setExportable(bool exportable);
        bool isExportable() const;

        // --- NEW: Geometry Handling with Polymorphism ---
        std::unique_ptr<IGeometry> setGeometry(std::unique_ptr<IGeometry> geometry);
        IGeometry* getGeometry() const;
        bool hasGeometry() const;
        const CadKernel::MeshBuffers& getMeshBuffers() const; // Caching version
        void invalidateMeshCache();
        void setMeshBuffers(CadKernel::MeshBuffers&& mesh); // Direct mesh update (for async remesh)
        void markMeshAsClean(); // Mark mesh cache as valid (keeps current mesh)

        // --- NEW: Mesh Groups for Materials ---
        mutable std::vector<MeshGroup> meshGroups;

        // --- NEW: Axis-Aligned Bounding Box ---
        mutable glm::vec3 aabbMin = glm::vec3(0.0f);
        mutable glm::vec3 aabbMax = glm::vec3(0.0f);
        mutable bool aabbValid = false;

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
        mutable std::unordered_map<unsigned int, std::set<unsigned int>> meshAdjacency;

        // --- PIMPL for the location map ---
        void buildLocationToVertexMap(); // Новый метод
        const TopoDS_Vertex* findVertexAtLocation(const glm::vec3& location) const; // Новый метод

        // --- NEW: Face adjacency data for fast selection ---
        int getFaceIdForTriangle(size_t triangleBaseIndex) const;
        const std::vector<size_t>* getFaceTriangles(int faceId) const;

        const std::vector<int>& getTriangleToFaceIDMap() const;
        const std::map<int, std::vector<size_t>>& getFacesMap() const;

        bool hasMesh() const; // Helper to check if cache is valid and not empty

    private:
        uint64_t id_;
        std::string name_;
        glm::mat4 transform_ = glm::mat4(1.0f);
        bool isExportable_ = true;
        
        // --- NEW: Polymorphic geometry representation ---
        std::unique_ptr<IGeometry> geometry_ = nullptr;
        
        // --- NEW: Render mesh cache ---
        mutable CadKernel::MeshBuffers mesh_buffers_cache_;
        mutable bool is_mesh_cache_dirty_ = true;

        // --- Face cache data (depends on mesh_buffers_cache_) ---
        mutable std::vector<int> triangleToFaceID_;
        mutable std::map<int, std::vector<size_t>> faces_;

        // PIMPL for vertex map (depends on BRepGeometry)
        std::unique_ptr<LocationToVertexMapImpl> locationToVertexMapPimpl_;
    };

} // namespace Urbaxio::Engine

#endif // URBAXIO_SCENE_OBJECT_H