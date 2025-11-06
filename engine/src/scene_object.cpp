#include "engine/scene_object.h"
#include "engine/geometry/BRepGeometry.h"
#include "engine/geometry/MeshGeometry.h"
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
        : id_(id), name_(std::move(name)), locationToVertexMapPimpl_(std::make_unique<LocationToVertexMapImpl>()) {
    }

    SceneObject::~SceneObject() = default;

    // Move constructor
    SceneObject::SceneObject(SceneObject&& other) noexcept
        : id_(other.id_),
        name_(std::move(other.name_)),
        transform_(other.transform_),
        isExportable_(other.isExportable_),
        geometry_(std::move(other.geometry_)),
        mesh_buffers_cache_(std::move(other.mesh_buffers_cache_)),
        is_mesh_cache_dirty_(other.is_mesh_cache_dirty_),
        meshGroups(std::move(other.meshGroups)),
        aabbMin(other.aabbMin), aabbMax(other.aabbMax), aabbValid(other.aabbValid),
        boundaryLineIDs(std::move(other.boundaryLineIDs)),
        meshAdjacency(std::move(other.meshAdjacency)),
        triangleToFaceID_(std::move(other.triangleToFaceID_)),
        faces_(std::move(other.faces_)),
        locationToVertexMapPimpl_(std::move(other.locationToVertexMapPimpl_))
    {
        other.id_ = 0;
        other.is_mesh_cache_dirty_ = true;
    }

    // Move assignment operator
    SceneObject& SceneObject::operator=(SceneObject&& other) noexcept {
        if (this != &other) {
            id_ = other.id_;
            name_ = std::move(other.name_);
            transform_ = other.transform_;
            isExportable_ = other.isExportable_;
            geometry_ = std::move(other.geometry_);
            mesh_buffers_cache_ = std::move(other.mesh_buffers_cache_);
            is_mesh_cache_dirty_ = other.is_mesh_cache_dirty_;
            meshGroups = std::move(other.meshGroups);
            aabbMin = other.aabbMin; aabbMax = other.aabbMax; aabbValid = other.aabbValid;
            boundaryLineIDs = std::move(other.boundaryLineIDs);
            meshAdjacency = std::move(other.meshAdjacency);
            triangleToFaceID_ = std::move(other.triangleToFaceID_);
            faces_ = std::move(other.faces_);
            locationToVertexMapPimpl_ = std::move(other.locationToVertexMapPimpl_);
            
            other.id_ = 0;
            other.is_mesh_cache_dirty_ = true;
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

    // --- Geometry & Mesh ---
    void SceneObject::setGeometry(std::unique_ptr<IGeometry> geometry) {
        geometry_ = std::move(geometry);
        invalidateMeshCache();

        buildLocationToVertexMap(); // Rebuild map, it might be a B-Rep shape
    }



    IGeometry* SceneObject::getGeometry() const {
        return geometry_.get();
    }



    bool SceneObject::hasGeometry() const {
        return geometry_ != nullptr;
    }



    void SceneObject::invalidateMeshCache() {
        is_mesh_cache_dirty_ = true;
        // Also clear dependent data
        mesh_buffers_cache_.clear();
        meshGroups.clear(); // This IS part of the cache and MUST be cleared.
        triangleToFaceID_.clear();
        faces_.clear();
        aabbValid = false;
        vao = 0; // Mark for GPU re-upload
    }



    // This is now the central point for getting renderable mesh data.
    // It uses a cache for performance.
    const CadKernel::MeshBuffers& SceneObject::getMeshBuffers() const {
        if (is_mesh_cache_dirty_) {
            // Recalculate mesh from the source geometry
            if (geometry_) {
                mesh_buffers_cache_ = geometry_->getRenderMesh();
            } else {
                mesh_buffers_cache_.clear();
            }



            // --- Post-processing after getting a new mesh ---
            meshAdjacency.clear();
            // --- MODIFICATION START: Preserve existing mesh groups ---
            bool hasExternalMeshGroups = !meshGroups.empty();
            if (!hasExternalMeshGroups) {
                meshGroups.clear();
            }
            // --- MODIFICATION END ---
            triangleToFaceID_.clear();
            faces_.clear();
            aabbValid = false;



            if (!mesh_buffers_cache_.isEmpty()) {
                // Calculate AABB, Mesh Groups, Adjacency, and Face Cache
                // (This is the logic moved from the old set_mesh_buffers)
                aabbMin = glm::vec3(std::numeric_limits<float>::max());
                aabbMax = glm::vec3(std::numeric_limits<float>::lowest());
                for (size_t i = 0; i < mesh_buffers_cache_.vertices.size(); i += 3) {
                    aabbMin.x = std::min(aabbMin.x, mesh_buffers_cache_.vertices[i]);
                    aabbMin.y = std::min(aabbMin.y, mesh_buffers_cache_.vertices[i+1]);
                    aabbMin.z = std::min(aabbMin.z, mesh_buffers_cache_.vertices[i+2]);
                    aabbMax.x = std::max(aabbMax.x, mesh_buffers_cache_.vertices[i]);
                    aabbMax.y = std::max(aabbMax.y, mesh_buffers_cache_.vertices[i+1]);
                    aabbMax.z = std::max(aabbMax.z, mesh_buffers_cache_.vertices[i+2]);
                }
                aabbValid = true;



                // --- MODIFICATION START: Create default group only if needed ---
                if (!hasExternalMeshGroups) {
                    MeshGroup defaultGroup;
                    defaultGroup.materialName = "Default";
                    defaultGroup.startIndex = 0;
                    defaultGroup.indexCount = mesh_buffers_cache_.indices.size();
                    if (defaultGroup.indexCount > 0) {
                        meshGroups.push_back(defaultGroup);
                    }
                }
                // --- MODIFICATION END ---
                
                for (size_t i = 0; i + 2 < mesh_buffers_cache_.indices.size(); i += 3) {
                    unsigned int i0 = mesh_buffers_cache_.indices[i];
                    unsigned int i1 = mesh_buffers_cache_.indices[i+1];
                    unsigned int i2 = mesh_buffers_cache_.indices[i+2];
                    meshAdjacency[i0].insert(i1); meshAdjacency[i0].insert(i2);
                    meshAdjacency[i1].insert(i0); meshAdjacency[i1].insert(i2);
                    meshAdjacency[i2].insert(i0); meshAdjacency[i2].insert(i1);
                }



                // Pre-calculate face adjacency
                const size_t triCount = mesh_buffers_cache_.indices.size() / 3;
                triangleToFaceID_.assign(triCount, -1);
                int currentFaceId = 0;
                std::map<std::pair<unsigned int, unsigned int>, std::vector<size_t>> edgeToTriangles;
                for (size_t i = 0; i < mesh_buffers_cache_.indices.size(); i += 3) {
                    unsigned int v_indices[3] = { mesh_buffers_cache_.indices[i], mesh_buffers_cache_.indices[i + 1], mesh_buffers_cache_.indices[i + 2] };
                    for (int j = 0; j < 3; ++j) {
                        unsigned int v1_idx = v_indices[j];
                        unsigned int v2_idx = v_indices[(j + 1) % 3];
                        if (v1_idx > v2_idx) std::swap(v1_idx, v2_idx);
                        edgeToTriangles[{v1_idx, v2_idx}].push_back(i);
                    }
                }
                
                for (size_t i = 0; i < mesh_buffers_cache_.indices.size(); i += 3) {
                    size_t triIndex = i / 3;
                    if (triangleToFaceID_[triIndex] != -1) continue;
                    
                    std::vector<size_t> currentFaceTriangles;
                    std::list<size_t> queue;
                    
                    unsigned int i0_start = mesh_buffers_cache_.indices[i];
                    glm::vec3 v0_start(mesh_buffers_cache_.vertices[i0_start*3], mesh_buffers_cache_.vertices[i0_start*3+1], mesh_buffers_cache_.vertices[i0_start*3+2]);
                    glm::vec3 refNormal(mesh_buffers_cache_.normals[i0_start*3], mesh_buffers_cache_.normals[i0_start*3+1], mesh_buffers_cache_.normals[i0_start*3+2]);
                    float refPlaneD = -glm::dot(refNormal, v0_start);
                    
                    queue.push_back(i);
                    triangleToFaceID_[triIndex] = currentFaceId;
                    while (!queue.empty()) {
                         size_t currentTriBaseIdx = queue.front();
                         queue.pop_front();
                         currentFaceTriangles.push_back(currentTriBaseIdx);
                         unsigned int current_v[3] = { mesh_buffers_cache_.indices[currentTriBaseIdx], mesh_buffers_cache_.indices[currentTriBaseIdx + 1], mesh_buffers_cache_.indices[currentTriBaseIdx + 2] };
                         for (int j = 0; j < 3; ++j) {
                            unsigned int v1 = current_v[j], v2 = current_v[(j+1)%3];
                            if (v1 > v2) std::swap(v1, v2);
                            const auto& neighbors = edgeToTriangles.at({v1, v2});
                            for (size_t neighborBaseIdx : neighbors) {
                                size_t neighborTriIdx = neighborBaseIdx / 3;
                                if (triangleToFaceID_[neighborTriIdx] == -1) {
                                    unsigned int n_i0 = mesh_buffers_cache_.indices[neighborBaseIdx];
                                    glm::vec3 n_v0(mesh_buffers_cache_.vertices[n_i0*3], mesh_buffers_cache_.vertices[n_i0*3+1], mesh_buffers_cache_.vertices[n_i0*3+2]);
                                    glm::vec3 neighborNorm(mesh_buffers_cache_.normals[n_i0*3], mesh_buffers_cache_.normals[n_i0*3+1], mesh_buffers_cache_.normals[n_i0*3+2]);
                                    if (glm::abs(glm::dot(refNormal, neighborNorm)) > 0.999f && glm::abs(glm::dot(refNormal, n_v0) + refPlaneD) < 1e-4f) {
                                        queue.push_back(neighborBaseIdx);
                                        triangleToFaceID_[neighborTriIdx] = currentFaceId;
                                    }
                                }
                            }
                         }
                    }
                    faces_[currentFaceId] = currentFaceTriangles;
                    currentFaceId++;
                }
            }



            is_mesh_cache_dirty_ = false;
        }
        return mesh_buffers_cache_;
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
        if (!hasGeometry()) return;



        // Try to cast geometry to BRepGeometry
        if (auto* brepGeom = dynamic_cast<BRepGeometry*>(geometry_.get())) {
            const TopoDS_Shape* shape = brepGeom->getShape();
            if (shape) {
                TopExp_Explorer explorer(*shape, TopAbs_VERTEX);
                for (; explorer.More(); explorer.Next()) {
                    TopoDS_Vertex vertex = TopoDS::Vertex(explorer.Current());
                    gp_Pnt p = BRep_Tool::Pnt(vertex);
                    glm::vec3 location(p.X(), p.Y(), p.Z());
                    locationToVertexMapPimpl_->theMap[location] = vertex;
                }
            }
        }
    }

    const TopoDS_Vertex* SceneObject::findVertexAtLocation(const glm::vec3& location) const {
        auto it = locationToVertexMapPimpl_->theMap.find(location);
        if (it != locationToVertexMapPimpl_->theMap.end()) {
            return &it->second;
        }
        return nullptr;
    }

    bool SceneObject::hasMesh() const {
        // This ensures the cache is populated before checking if it's empty.
        // It calls the main caching getter first.
        getMeshBuffers(); 
        return !mesh_buffers_cache_.isEmpty();
    }

} // namespace Urbaxio::Engine