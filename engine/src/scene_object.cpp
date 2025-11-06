#include "engine/scene_object.h"
#include <utility>
#include <iostream>
#include <map>
#include <list>
#include <limits> // For std::numeric_limits
#include <TopoDS.hxx> // <-- ГЛАВНОЕ ИСПРАВЛЕНИЕ: этот инклуд был пропущен
#include <TopoDS_Shape.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs.hxx>
#include <BRep_Tool.hxx>
#include <gp_Pnt.hxx>
#include "engine/MeshGroup.h"

namespace Urbaxio::Engine {

    // Определение структуры PIMPL
    struct LocationToVertexMapImpl {
        std::map<glm::vec3, TopoDS_Vertex, Urbaxio::Vec3Comparator> theMap;
    };

    SceneObject::SceneObject(uint64_t id, std::string name)
        : id_(id), name_(std::move(name)), shape_(nullptr), locationToVertexMapPimpl_(std::make_unique<LocationToVertexMapImpl>()) {
    }

    SceneObject::~SceneObject() = default;

    // Конструктор перемещения
    SceneObject::SceneObject(SceneObject&& other) noexcept
        : id_(other.id_),
        name_(std::move(other.name_)),
        transform_(other.transform_),
        isExportable_(other.isExportable_),
        shape_(std::move(other.shape_)),
        mesh_buffers_(std::move(other.mesh_buffers_)),
        meshGroups(std::move(other.meshGroups)),
        aabbMin(other.aabbMin), aabbMax(other.aabbMax), aabbValid(other.aabbValid),
        boundaryLineIDs(std::move(other.boundaryLineIDs)), // <-- ДОБАВИТЬ
        meshAdjacency(std::move(other.meshAdjacency)),
        locationToVertexMapPimpl_(std::move(other.locationToVertexMapPimpl_)) // <-- ИЗМЕНИТЬ
    {
        other.id_ = 0;
    }

    // Оператор присваивания перемещением
    SceneObject& SceneObject::operator=(SceneObject&& other) noexcept {
        if (this != &other) {
            id_ = other.id_;
            name_ = std::move(other.name_);
            transform_ = other.transform_;
            isExportable_ = other.isExportable_;
            shape_ = std::move(other.shape_);
            mesh_buffers_ = std::move(other.mesh_buffers_);
            meshGroups = std::move(other.meshGroups);
            aabbMin = other.aabbMin; aabbMax = other.aabbMax; aabbValid = other.aabbValid;
            boundaryLineIDs = std::move(other.boundaryLineIDs); // <-- ДОБАВИТЬ
            meshAdjacency = std::move(other.meshAdjacency);
            locationToVertexMapPimpl_ = std::move(other.locationToVertexMapPimpl_); // <-- ИЗМЕНИТЬ
            other.id_ = 0;
        }
        return *this;
    }

    uint64_t SceneObject::get_id() const { return id_; }
    const std::string& SceneObject::get_name() const { return name_; }
    void SceneObject::set_name(const std::string& name) { name_ = name; }

    // --- NEW: Transform methods ---
    void SceneObject::setTransform(const glm::mat4& newTransform) {
        transform_ = newTransform;
    }
    const glm::mat4& SceneObject::getTransform() const {
        return transform_;
    }

    // --- NEW: Exportability ---
    void SceneObject::setExportable(bool exportable) { isExportable_ = exportable; }
    bool SceneObject::isExportable() const { return isExportable_; }

    // --- B-Rep Shape ---
    void SceneObject::set_shape(Urbaxio::CadKernel::OCCT_ShapeUniquePtr shape) {
        shape_ = std::move(shape);
        buildLocationToVertexMap(); // Обновляем карту при смене формы
    }
    const TopoDS_Shape* SceneObject::get_shape() const { return shape_.get(); }
    bool SceneObject::has_shape() const { return shape_ != nullptr; }

    // --- Visual Mesh ---
    void SceneObject::set_mesh_buffers(Urbaxio::CadKernel::MeshBuffers buffers) {
        mesh_buffers_ = std::move(buffers);
        
        meshAdjacency.clear();
        meshGroups.clear();
        // --- NEW: Clear face cache ---
        triangleToFaceID_.clear();
        faces_.clear();
        aabbValid = false;
        if (mesh_buffers_.isEmpty()) {
            return;
        }
        
        // --- Calculate AABB ---
        if (!mesh_buffers_.vertices.empty()) {
            aabbMin = glm::vec3(std::numeric_limits<float>::max());
            aabbMax = glm::vec3(std::numeric_limits<float>::lowest());
            for (size_t i = 0; i < mesh_buffers_.vertices.size(); i += 3) {
                aabbMin.x = std::min(aabbMin.x, mesh_buffers_.vertices[i]);
                aabbMin.y = std::min(aabbMin.y, mesh_buffers_.vertices[i+1]);
                aabbMin.z = std::min(aabbMin.z, mesh_buffers_.vertices[i+2]);
                aabbMax.x = std::max(aabbMax.x, mesh_buffers_.vertices[i]);
                aabbMax.y = std::max(aabbMax.y, mesh_buffers_.vertices[i+1]);
                aabbMax.z = std::max(aabbMax.z, mesh_buffers_.vertices[i+2]);
            }
            aabbValid = true;
        }
        
        // --- REVERTED/FIXED: This function MUST create a default material group ---
        // Create a single default group that covers the entire mesh.
        // This ensures the object is always renderable.
        // Importers (like OBJ) can overwrite this with more specific groups.
        MeshGroup defaultGroup;
        defaultGroup.materialName = "Default";
        defaultGroup.startIndex = 0;
        defaultGroup.indexCount = mesh_buffers_.indices.size();
        if (defaultGroup.indexCount > 0) {
            meshGroups.push_back(defaultGroup);
        }
        for (size_t i = 0; i + 2 < mesh_buffers_.indices.size(); i += 3) {
            unsigned int i0 = mesh_buffers_.indices[i];
            unsigned int i1 = mesh_buffers_.indices[i+1];
            unsigned int i2 = mesh_buffers_.indices[i+2];
            // Add edges to the map. (i0, i1), (i1, i2), (i2, i0)
            meshAdjacency[i0].insert(i1);
            meshAdjacency[i0].insert(i2);
            
            meshAdjacency[i1].insert(i0);
            meshAdjacency[i1].insert(i2);
            
            meshAdjacency[i2].insert(i0);
            meshAdjacency[i2].insert(i1);
        }

        // --- NEW: Pre-calculate face adjacency for fast selection ---
        if (mesh_buffers_.indices.empty()) {
            return;
        }
        
        // 1. Build an edge-to-triangle map for adjacency lookups ONCE.
        std::map<std::pair<unsigned int, unsigned int>, std::vector<size_t>> edgeToTriangles;
        for (size_t i = 0; i < mesh_buffers_.indices.size(); i += 3) {
            unsigned int v_indices[3] = { mesh_buffers_.indices[i], mesh_buffers_.indices[i + 1], mesh_buffers_.indices[i + 2] };
            for (int j = 0; j < 3; ++j) {
                unsigned int v1_idx = v_indices[j];
                unsigned int v2_idx = v_indices[(j + 1) % 3];
                if (v1_idx > v2_idx) std::swap(v1_idx, v2_idx);
                edgeToTriangles[{v1_idx, v2_idx}].push_back(i);
            }
        }

        // 2. Iterate through all triangles and group them into faces using BFS.
        const size_t triCount = mesh_buffers_.indices.size() / 3;
        triangleToFaceID_.assign(triCount, -1);
        int currentFaceId = 0;
        
        for (size_t i = 0; i < mesh_buffers_.indices.size(); i += 3) {
            size_t triIndex = i / 3;
            if (triangleToFaceID_[triIndex] != -1) {
                continue; // Already processed
            }
            std::vector<size_t> currentFaceTriangles;
            std::list<size_t> queue;
            
            // Define the reference plane from the starting triangle
            unsigned int i0 = mesh_buffers_.indices[i];
            glm::vec3 v0(mesh_buffers_.vertices[i0*3], mesh_buffers_.vertices[i0*3+1], mesh_buffers_.vertices[i0*3+2]);
            glm::vec3 referenceNormal(mesh_buffers_.normals[i0*3], mesh_buffers_.normals[i0*3+1], mesh_buffers_.normals[i0*3+2]);
            float referencePlaneD = -glm::dot(referenceNormal, v0);

            // Start BFS
            queue.push_back(i);
            triangleToFaceID_[triIndex] = currentFaceId;
            while (!queue.empty()) {
                size_t currentTriangleBaseIndex = queue.front();
                queue.pop_front();
                currentFaceTriangles.push_back(currentTriangleBaseIndex);
                unsigned int current_v_indices[3] = { mesh_buffers_.indices[currentTriangleBaseIndex], mesh_buffers_.indices[currentTriangleBaseIndex + 1], mesh_buffers_.indices[currentTriangleBaseIndex + 2] };

                // Check neighbors through all 3 edges
                for (int j = 0; j < 3; ++j) {
                    unsigned int v1_idx = current_v_indices[j];
                    unsigned int v2_idx = current_v_indices[(j + 1) % 3];
                    if (v1_idx > v2_idx) std::swap(v1_idx, v2_idx);
                    const auto& potentialNeighbors = edgeToTriangles.at({v1_idx, v2_idx});
                    for (size_t neighborBaseIndex : potentialNeighbors) {
                        size_t neighborTriIndex = neighborBaseIndex / 3;
                        if (triangleToFaceID_[neighborTriIndex] == -1) {
                             unsigned int n_i0 = mesh_buffers_.indices[neighborBaseIndex];
                             glm::vec3 n_v0(mesh_buffers_.vertices[n_i0*3], mesh_buffers_.vertices[n_i0*3+1], mesh_buffers_.vertices[n_i0*3+2]);
                             glm::vec3 neighborNormal(mesh_buffers_.normals[n_i0*3], mesh_buffers_.normals[n_i0*3+1], mesh_buffers_.normals[n_i0*3+2]);
                            
                             const float NORMAL_DOT_TOLERANCE = 0.999f;
                             const float PLANE_DIST_TOLERANCE = 1e-4f;
                             if (glm::abs(glm::dot(referenceNormal, neighborNormal)) > NORMAL_DOT_TOLERANCE) {
                                 if (glm::abs(glm::dot(referenceNormal, n_v0) + referencePlaneD) < PLANE_DIST_TOLERANCE) {
                                     queue.push_back(neighborBaseIndex);
                                     triangleToFaceID_[neighborTriIndex] = currentFaceId;
                                 }
                             }
                        }
                    }
                }
            }
            faces_[currentFaceId] = currentFaceTriangles;
            currentFaceId++;
        }
    }
    const Urbaxio::CadKernel::MeshBuffers& SceneObject::get_mesh_buffers() const {
        return mesh_buffers_;
    }
    bool SceneObject::has_mesh() const {
        return !mesh_buffers_.isEmpty();
    }

    // --- NEW: Face cache accessors ---
    int SceneObject::getFaceIdForTriangle(size_t triangleBaseIndex) const {
        size_t triIndex = triangleBaseIndex / 3;
        if (triIndex < triangleToFaceID_.size()) {
            return triangleToFaceID_[triIndex];
        }
        return -1;
    }

    const std::vector<size_t>* SceneObject::getFaceTriangles(int faceId) const {
        auto it = faces_.find(faceId);
        if (it != faces_.end()) {
            return &it->second;
        }
        return nullptr;
    }

    // --- START OF MODIFICATION ---
    const std::vector<int>& SceneObject::getTriangleToFaceIDMap() const {
        return triangleToFaceID_;
    }

    const std::map<int, std::vector<size_t>>& SceneObject::getFacesMap() const {
        return faces_;
    }
    // --- END OF MODIFICATION ---

    // --- РЕАЛИЗАЦИЯ НОВЫХ МЕТОДОВ ---
    void SceneObject::buildLocationToVertexMap() {
        locationToVertexMapPimpl_->theMap.clear();
        if (!has_shape()) return;

        TopExp_Explorer explorer(*shape_, TopAbs_VERTEX);
        for (; explorer.More(); explorer.Next()) {
            TopoDS_Vertex vertex = TopoDS::Vertex(explorer.Current());
            gp_Pnt p = BRep_Tool::Pnt(vertex);
            glm::vec3 location(p.X(), p.Y(), p.Z());
            locationToVertexMapPimpl_->theMap[location] = vertex;
        }
    }

    const TopoDS_Vertex* SceneObject::findVertexAtLocation(const glm::vec3& location) const {
        auto it = locationToVertexMapPimpl_->theMap.find(location);
        if (it != locationToVertexMapPimpl_->theMap.end()) {
            return &it->second;
        }
        return nullptr;
    }

} // namespace Urbaxio::Engine