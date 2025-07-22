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
    const std::vector<glm::vec3>& faceVertices,
    const glm::vec3& faceNormal,
    float distance,
    bool disableMerge)
    : scene_(scene), 
      faceVertices_(faceVertices),
      faceNormal_(faceNormal),
      distance_(distance),
      disableMerge_(disableMerge)
{
}

const char* PushPullCommand::GetName() const {
    return "Push/Pull";
}

void PushPullCommand::Execute() {
    if (isExecuted_) { // This is a REDO operation
        if (!shapeAfter_.empty() && targetObjectId_ != 0) {
            RestoreObjectShape(targetObjectId_, shapeAfter_);
        }
        return;
    }

    // This is the FIRST execution of the command
    SceneObject* object = scene_->FindObjectByFace(faceVertices_);
    if (!object) {
        std::cerr << "PushPullCommand Error: Could not find an object containing the specified face to execute on." << std::endl;
        return;
    }
    
    targetObjectId_ = object->get_id(); // Store the ID we found for Undo/Redo
    shapeBefore_ = SerializeShape(*object->get_shape());
    if (shapeBefore_.empty()) {
        std::cerr << "PushPullCommand Error: Failed to serialize 'before' state." << std::endl;
        return;
    }
    
    bool success = scene_->ExtrudeFace(targetObjectId_, faceVertices_, faceNormal_, distance_, disableMerge_);
    
    if (success) {
        shapeAfter_ = SerializeShape(*object->get_shape());
    } else {
        std::cerr << "PushPullCommand: ExtrudeFace operation failed. Aborting." << std::endl;
        RestoreObjectShape(targetObjectId_, shapeBefore_);
    }

    isExecuted_ = true;
}

void PushPullCommand::Undo() {
    if (shapeBefore_.empty() || targetObjectId_ == 0) {
        std::cerr << "PushPullCommand Error: Cannot undo, 'before' state was not properly captured." << std::endl;
        return;
    }
    RestoreObjectShape(targetObjectId_, shapeBefore_);
}

void PushPullCommand::RestoreObjectShape(uint64_t objectId, const std::vector<char>& shapeData) {
    SceneObject* object = scene_->get_object_by_id(objectId);
    if (!object) {
        std::cerr << "PushPullCommand Warning: Could not find object " << objectId << " to restore shape." << std::endl;
        return;
    }
    
    TopoDS_Shape restoredShape;
    std::stringstream ss(std::string(shapeData.begin(), shapeData.end()));
    try {
        BinTools::Read(restoredShape, ss);
    } catch (...) {
        std::cerr << "PushPullCommand Error: Failed to deserialize shape data." << std::endl;
        return;
    }
    
    if (restoredShape.IsNull()) {
        std::cerr << "PushPullCommand Error: Deserialized shape is null." << std::endl;
        return;
    }

    object->set_shape(Urbaxio::CadKernel::OCCT_ShapeUniquePtr(new TopoDS_Shape(restoredShape)));
    scene_->UpdateObjectBoundary(object);
    object->set_mesh_buffers(Urbaxio::CadKernel::TriangulateShape(*object->get_shape()));
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