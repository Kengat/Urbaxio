#include "engine/scene_object.h"
#include <utility>
#include <iostream>
#include <map>
#include <TopoDS.hxx> // <-- ГЛАВНОЕ ИСПРАВЛЕНИЕ: этот инклуд был пропущен
#include <TopoDS_Shape.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs.hxx>
#include <BRep_Tool.hxx>
#include <gp_Pnt.hxx>

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
        shape_(std::move(other.shape_)),
        mesh_buffers_(std::move(other.mesh_buffers_)),
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
            shape_ = std::move(other.shape_);
            mesh_buffers_ = std::move(other.mesh_buffers_);
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
        
        // --- NEW: Build the vertex adjacency map ---
        meshAdjacency.clear();
        if (mesh_buffers_.isEmpty()) {
            return;
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
    }
    const Urbaxio::CadKernel::MeshBuffers& SceneObject::get_mesh_buffers() const {
        return mesh_buffers_;
    }
    bool SceneObject::has_mesh() const {
        return !mesh_buffers_.isEmpty();
    }

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