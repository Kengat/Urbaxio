#include "engine/scene_object.h"
#include <utility>
#include <iostream>
#include <TopoDS_Shape.hxx>

namespace Urbaxio::Engine {

    SceneObject::SceneObject(uint64_t id, std::string name)
        : id_(id), name_(std::move(name)), shape_(nullptr) {
    }

    SceneObject::~SceneObject() {}

    //                                    
    SceneObject::SceneObject(SceneObject&& other) noexcept
        : id_(other.id_),
        name_(std::move(other.name_)),
        shape_(std::move(other.shape_)),
        mesh_buffers_(std::move(other.mesh_buffers_)) // <---               
    {
        other.id_ = 0;
    }

    //                                               
    SceneObject& SceneObject::operator=(SceneObject&& other) noexcept {
        if (this != &other) {
            id_ = other.id_;
            name_ = std::move(other.name_);
            shape_ = std::move(other.shape_);
            mesh_buffers_ = std::move(other.mesh_buffers_); // <---               
            other.id_ = 0;
        }
        return *this;
    }

    uint64_t SceneObject::get_id() const { return id_; }
    const std::string& SceneObject::get_name() const { return name_; }
    void SceneObject::set_name(const std::string& name) { name_ = name; }

    // ---           BRep ---
    void SceneObject::set_shape(Urbaxio::CadKernel::OCCT_ShapeUniquePtr shape) { shape_ = std::move(shape); }
    const TopoDS_Shape* SceneObject::get_shape() const { return shape_.get(); }
    bool SceneObject::has_shape() const { return shape_ != nullptr; }

    // ---                ---
    void SceneObject::set_mesh_buffers(Urbaxio::CadKernel::MeshBuffers buffers) { // <---           
        mesh_buffers_ = std::move(buffers); //                                     
    }
    const Urbaxio::CadKernel::MeshBuffers& SceneObject::get_mesh_buffers() const { // <---           
        return mesh_buffers_;
    }
    bool SceneObject::has_mesh() const { // <---           
        return !mesh_buffers_.isEmpty();
    }

} // namespace Urbaxio::Engine