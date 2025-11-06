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

    // --- 1. Transform the CAD B-Rep shape (if it exists) ---
    if (object->has_shape()) {
        try {
            gp_Trsf occtTransform;
            occtTransform.SetTranslation(gp_Vec(vector.x, vector.y, vector.z));
            
            BRepBuilderAPI_Transform transformer(*object->get_shape(), occtTransform);
            transformer.Build();

            if (transformer.IsDone()) {
                // Update the B-Rep shape
                object->set_shape(Urbaxio::CadKernel::OCCT_ShapeUniquePtr(new TopoDS_Shape(transformer.Shape())));
                // Re-triangulate from the new shape to update the mesh buffers
                object->set_mesh_buffers(Urbaxio::CadKernel::TriangulateShape(*object->get_shape()));
                object->vao = 0; // Mark for re-upload if it's dynamic
            } else {
                std::cerr << "MoveCommand Warning: OCCT transformation failed for object " << objectId_ << std::endl;
            }
        } catch(...) {
            std::cerr << "MoveCommand Warning: OCCT exception during transformation for object " << objectId_ << std::endl;
        }
    } else if (object->has_mesh()) {
        // Fallback for non-BRep objects: manually transform vertices
        Urbaxio::CadKernel::MeshBuffers& mesh = const_cast<Urbaxio::CadKernel::MeshBuffers&>(object->get_mesh_buffers());
        for (size_t i = 0; i < mesh.vertices.size(); i += 3) {
            mesh.vertices[i] += vector.x;
            mesh.vertices[i+1] += vector.y;
            mesh.vertices[i+2] += vector.z;
        }
        object->vao = 0;
    }

    // --- 2. Update the boundary lines based on the new shape/mesh ---
    scene_->UpdateObjectBoundary(object);

    // --- 3. Mark the static scene as dirty to trigger recompilation ---
    scene_->MarkStaticGeometryDirty();
}

} // namespace Urbaxio::Engine