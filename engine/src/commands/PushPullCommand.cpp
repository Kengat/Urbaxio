#include "engine/commands/PushPullCommand.h"
#include "engine/scene.h"
#include "engine/scene_object.h"
#include <cad_kernel/cad_kernel.h>
#include <cad_kernel/MeshBuffers.h>

#include <TopoDS_Shape.hxx>
#include <BinTools.hxx>
#include <sstream>
#include <iostream>

namespace Urbaxio::Engine {

PushPullCommand::PushPullCommand(
    Scene* scene, 
    uint64_t objectId, 
    const std::vector<size_t>& faceIndices, 
    const glm::vec3& direction, 
    float distance,
    bool disableMerge)
    : scene_(scene), 
      objectId_(objectId), 
      faceIndices_(faceIndices), 
      direction_(direction), 
      distance_(distance),
      disableMerge_(disableMerge)
{
}

const char* PushPullCommand::GetName() const {
    return "Push/Pull";
}

void PushPullCommand::Execute() {
    // If the command was already executed, we are in a "Redo" case.
    // We can simply restore the "after" state.
    if (isExecuted_ && !shapeAfter_.empty()) {
        RestoreObjectShape(shapeAfter_);
        return;
    }

    // This is the first time executing the command.
    SceneObject* object = scene_->get_object_by_id(objectId_);
    if (!object || !object->has_shape()) {
        std::cerr << "PushPullCommand Error: Could not find object " << objectId_ << " to execute on." << std::endl;
        return;
    }

    // Memento: Capture the state BEFORE modification.
    shapeBefore_ = SerializeShape(*object->get_shape());
    if (shapeBefore_.empty()) {
        std::cerr << "PushPullCommand Error: Failed to serialize 'before' state." << std::endl;
        return;
    }

    // Perform the actual geometric operation.
    bool success = scene_->ExtrudeFace(objectId_, faceIndices_, direction_, distance_, disableMerge_);

    if (success) {
        // Memento: Capture the state AFTER modification.
        shapeAfter_ = SerializeShape(*object->get_shape());
        if (shapeAfter_.empty()) {
            std::cerr << "PushPullCommand Warning: Failed to serialize 'after' state. Undo may not work correctly." << std::endl;
        }
    } else {
        std::cerr << "PushPullCommand: ExtrudeFace operation failed. Aborting command." << std::endl;
        // Since the operation failed, we should restore the "before" state immediately.
        RestoreObjectShape(shapeBefore_);
    }

    isExecuted_ = true;
}

void PushPullCommand::Undo() {
    if (shapeBefore_.empty()) {
        std::cerr << "PushPullCommand Error: Cannot undo, 'before' state was never captured." << std::endl;
        return;
    }
    // Memento: Restore the "before" state.
    RestoreObjectShape(shapeBefore_);
}

void PushPullCommand::RestoreObjectShape(const std::vector<char>& shapeData) {
    SceneObject* object = scene_->get_object_by_id(objectId_);
    if (!object) {
        std::cerr << "PushPullCommand Error: Could not find object " << objectId_ << " to restore shape." << std::endl;
        return;
    }

    TopoDS_Shape restoredShape;
    std::stringstream ss(std::string(shapeData.begin(), shapeData.end()));
    try {
        BinTools::Read(restoredShape, ss);
    } catch (...) {
        std::cerr << "PushPullCommand Error: Failed to deserialize shape data during restore." << std::endl;
        return;
    }
    
    if (restoredShape.IsNull()) {
        std::cerr << "PushPullCommand Error: Deserialized shape is null." << std::endl;
        return;
    }

    // Replace the object's shape and update all derived data (lines, mesh).
    object->set_shape(Urbaxio::CadKernel::OCCT_ShapeUniquePtr(new TopoDS_Shape(restoredShape)));
    scene_->UpdateObjectBoundary(object);
    object->set_mesh_buffers(Urbaxio::CadKernel::TriangulateShape(*object->get_shape()));
    
    // Mark GPU resources as dirty so they will be re-uploaded.
    object->vao = 0;
}

std::vector<char> PushPullCommand::SerializeShape(const TopoDS_Shape& shape) {
    std::stringstream ss;
    try {
        BinTools::Write(shape, ss);
        std::string const& s = ss.str();
        return std::vector<char>(s.begin(), s.end());
    } catch (...) {
        std::cerr << "PushPullCommand Error: Exception during shape serialization." << std::endl;
        return {};
    }
}

} // namespace Urbaxio::Engine 