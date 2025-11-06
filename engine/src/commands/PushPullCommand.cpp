#include "engine/commands/PushPullCommand.h"
#include "engine/scene.h"
#include "engine/scene_object.h"
#include "engine/geometry/BRepGeometry.h"
#include "engine/geometry/MeshGeometry.h"
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
    
    // NEW: Get shape via BRepGeometry
    auto* brepGeom = dynamic_cast<BRepGeometry*>(object->getGeometry());
    if (!brepGeom || !brepGeom->getShape()) {
        std::cerr << "PushPullCommand Error: Target object is not a valid B-Rep." << std::endl;
        return;
    }

    targetObjectId_ = object->get_id();
    shapeBefore_ = SerializeShape(*brepGeom->getShape());
    if (shapeBefore_.empty()) {
        std::cerr << "PushPullCommand Error: Failed to serialize 'before' state." << std::endl;
        return;
    }
    
    bool success = scene_->ExtrudeFace(targetObjectId_, faceVertices_, faceNormal_, distance_, disableMerge_);
    
    if (success) {
        // After success, re-fetch the geometry and serialize the new shape
        auto* updatedGeom = dynamic_cast<BRepGeometry*>(object->getGeometry());
        if(updatedGeom && updatedGeom->getShape()) {
            shapeAfter_ = SerializeShape(*updatedGeom->getShape());
        }
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

    // NEW: Update via the new geometry system
    auto geometry = std::make_unique<BRepGeometry>(CadKernel::OCCT_ShapeUniquePtr(new TopoDS_Shape(restoredShape)));
    object->setGeometry(std::move(geometry));
    scene_->UpdateObjectBoundary(object);

    // Mark the scene dirty to force the renderer to re-compile the static batch
    scene_->MarkStaticGeometryDirty();
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