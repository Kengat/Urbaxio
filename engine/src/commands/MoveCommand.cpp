#include "engine/commands/MoveCommand.h"
#include "engine/scene.h"
#include "engine/scene_object.h"
#include "engine/line.h"
#include "engine/geometry/BRepGeometry.h"
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

    // --- 1. Transform the geometry source (if it's a B-Rep) ---
    if (auto* brepGeom = dynamic_cast<BRepGeometry*>(object->getGeometry())) {
        const TopoDS_Shape* shape = brepGeom->getShape();
        if (shape) {
            try {
                gp_Trsf occtTransform;
                occtTransform.SetTranslation(gp_Vec(vector.x, vector.y, vector.z));
                BRepBuilderAPI_Transform transformer(*shape, occtTransform);
                transformer.Build();
                if (transformer.IsDone()) {
                    brepGeom->setShape(CadKernel::OCCT_ShapeUniquePtr(new TopoDS_Shape(transformer.Shape())));
                    object->invalidateMeshCache();
                } else {
                    std::cerr << "MoveCommand Warning: OCCT transformation failed for object " << objectId_ << std::endl;
                }
            } catch(...) {
                std::cerr << "MoveCommand Warning: OCCT exception during transformation for object " << objectId_ << std::endl;
            }
        }
    }
    // Note: Moving pure mesh or volumetric objects is not supported by this command yet.
    // It would require manually transforming vertices in their respective geometry classes.

    // --- 2. Update the boundary lines based on the new shape ---
    scene_->UpdateObjectBoundary(object);

    // --- 3. Mark the static scene as dirty to trigger recompilation ---
    scene_->MarkStaticGeometryDirty();
}

} // namespace Urbaxio::Engine