#include "engine/commands/SculptCommand.h"
#include "engine/scene.h"
#include "engine/scene_object.h"
#include "engine/geometry/VolumetricGeometry.h"
#include "engine/geometry/VoxelGrid.h"
#include <iostream>

namespace Urbaxio::Engine {

SculptCommand::SculptCommand(Scene* scene, uint64_t objectId, 
                             std::vector<float>&& dataBefore, 
                             std::vector<float>&& dataAfter)
    : scene_(scene), 
      objectId_(objectId), 
      dataBefore_(std::move(dataBefore)), 
      dataAfter_(std::move(dataAfter)) 
{}

const char* SculptCommand::GetName() const {
    return "Sculpt Stroke";
}

void SculptCommand::Execute() {
    std::cout << "[Command] Executing Sculpt Stroke on object " << objectId_ << std::endl;
    applyGridData(dataAfter_);
}

void SculptCommand::Undo() {
    std::cout << "[Command] Undoing Sculpt Stroke on object " << objectId_ << std::endl;
    applyGridData(dataBefore_);
}

void SculptCommand::applyGridData(const std::vector<float>& data) {
    if (!scene_) return;
    SceneObject* obj = scene_->get_object_by_id(objectId_);
    if (!obj) return;

    auto* volGeom = dynamic_cast<VolumetricGeometry*>(obj->getGeometry());
    if (!volGeom) return;

    VoxelGrid* grid = volGeom->getGrid();
    if (!grid) return;
    
    size_t expectedSize = grid->dimensions.x * grid->dimensions.y * grid->dimensions.z;
    if (data.size() == expectedSize) {
        // Convert dense array back to sparse OpenVDB grid
        grid->fromDenseArray(data);
        
        // Don't invalidate mesh cache - keep old mesh until async update arrives
        // This prevents sync Marching Cubes during Undo/Redo
        
        std::cout << "[SculptCommand] Restored grid state. Active voxels: " 
                  << grid->getActiveVoxelCount() << " (mesh update will be async)" << std::endl;
    } else {
        std::cerr << "SculptCommand Error: Grid data size mismatch! Expected " 
                  << expectedSize << ", got " << data.size() << std::endl;
    }
}

} // namespace Urbaxio::Engine

