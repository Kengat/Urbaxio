#include "engine/commands/SculptCommand.h"
#include "engine/scene.h"
#include "engine/scene_object.h"
#include "engine/geometry/VolumetricGeometry.h"
#include "engine/geometry/VoxelGrid.h"
#include <iostream>

namespace Urbaxio::Engine {

// MODIFIED: Constructor now accepts and stores bounding boxes
SculptCommand::SculptCommand(Scene* scene, uint64_t objectId, 
                             std::vector<float>&& dataBefore, 
                             std::vector<float>&& dataAfter,
                             openvdb::CoordBBox beforeBBox,
                             openvdb::CoordBBox afterBBox)
    : scene_(scene), 
      objectId_(objectId), 
      dataBefore_(std::move(dataBefore)), 
      dataAfter_(std::move(dataAfter)),
      beforeBBox_(beforeBBox),
      afterBBox_(afterBBox)
{}

const char* SculptCommand::GetName() const {
    return "Sculpt Stroke";
}

void SculptCommand::Execute() {
    std::cout << "[Command] Executing Sculpt Stroke on object " << objectId_ << std::endl;
    applyGridData(dataAfter_, afterBBox_);
}

void SculptCommand::Undo() {
    std::cout << "[Command] Undoing Sculpt Stroke on object " << objectId_ << std::endl;
    applyGridData(dataBefore_, beforeBBox_);
}

void SculptCommand::applyGridData(const std::vector<float>& data, const openvdb::CoordBBox& bbox) {
    if (!scene_) return;
    SceneObject* obj = scene_->get_object_by_id(objectId_);
    if (!obj) return;

    auto* volGeom = dynamic_cast<VolumetricGeometry*>(obj->getGeometry());
    if (!volGeom) return;

    VoxelGrid* grid = volGeom->getGrid();
    if (!grid) return;
    
    // NEW: Use the fromDenseArray overload that takes a bounding box
    grid->fromDenseArray(data, bbox);
    
    std::cout << "[SculptCommand] ✅ Restored grid state. Active voxels: " 
                << grid->getActiveVoxelCount() << std::endl;
    
    // Mark GPU as dirty so it will be re-uploaded
    grid->gpuDirty_ = true;
}

} // namespace Urbaxio::Engine

