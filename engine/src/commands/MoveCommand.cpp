#include "engine/commands/MoveCommand.h"
#include "engine/scene.h"
#include "engine/scene_object.h"
#include "engine/line.h"
#include "cad_kernel/MeshBuffers.h" // <-- FIX: Include full definition
#include "cad_kernel/cad_kernel.h"   // <-- FIX: Include full definition

// OCCT Includes for transformation
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <TopoDS_Shape.hxx>

#include <iostream>
#include <vector>

namespace Urbaxio::Engine {

MoveCommand::MoveCommand(Scene* scene, uint64_t objectId, const glm::vec3& translationVector)
    : scene_(scene), objectId_(objectId), translationVector_(translationVector) {}

const char* MoveCommand::GetName() const {
    return "Move Object";
}

void MoveCommand::Execute() {
    applyTransform(translationVector_);
}

void MoveCommand::Undo() {
    applyTransform(-translationVector_);
}

void MoveCommand::applyTransform(const glm::vec3& vector) {
    if (!scene_ || objectId_ == 0) {
        return;
    }

    SceneObject* object = scene_->get_object_by_id(objectId_);
    if (!object) {
        std::cerr << "MoveCommand Error: Object with ID " << objectId_ << " not found." << std::endl;
        return;
    }

    // --- 1. Transform the CAD B-Rep shape ---
    if (object->has_shape()) {
        try {
            gp_Trsf occtTransform;
            occtTransform.SetTranslation(gp_Vec(vector.x, vector.y, vector.z));
            
            // --- FIX: Correct constructor call ---
            BRepBuilderAPI_Transform transformer(*object->get_shape(), occtTransform);
            transformer.Build();

            if (transformer.IsDone()) {
                object->set_shape(Urbaxio::CadKernel::OCCT_ShapeUniquePtr(new TopoDS_Shape(transformer.Shape())));
            } else {
                std::cerr << "MoveCommand Warning: OCCT transformation failed for object " << objectId_ << std::endl;
            }
        } catch(...) {
            std::cerr << "MoveCommand Warning: OCCT exception during transformation for object " << objectId_ << std::endl;
        }
    }
    
    // --- 2. Transform the visual mesh (much faster than re-triangulating) ---
    if (object->has_mesh()) {
        // We need a non-const reference to modify the mesh buffers
        Urbaxio::CadKernel::MeshBuffers& mesh = const_cast<Urbaxio::CadKernel::MeshBuffers&>(object->get_mesh_buffers());
        for (size_t i = 0; i < mesh.vertices.size(); i += 3) {
            mesh.vertices[i] += vector.x;
            mesh.vertices[i+1] += vector.y;
            mesh.vertices[i+2] += vector.z;
        }
        // Mark the GPU buffers as dirty so the main loop re-uploads them
        object->vao = 0;
    }

    // --- 3. Transform the associated boundary lines ---
    // This will rebuild the vertex adjacency map correctly by removing old lines and adding new ones.
    scene_->UpdateObjectBoundary(object);
}

} // namespace Urbaxio::Engine