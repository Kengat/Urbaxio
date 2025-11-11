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
    
    // NEW: Check if data size matches expected size
    // The data array should match the logical dimensions
    size_t expectedSize = grid->dimensions.x * grid->dimensions.y * grid->dimensions.z;
    
    if (data.size() == expectedSize) {
        // LEGACY path: full dimensions array
        grid->fromDenseArray(data);
        std::cout << "[SculptCommand] ✅ Restored grid state (legacy format). Active voxels: " 
                  << grid->getActiveVoxelCount() << std::endl;
    } else if (data.size() > 0) {
        // NEW path: Try to interpret as bounded region data
        // This requires storing bbox info in the command, which we don't have yet
        // For now, just log error
        std::cerr << "[SculptCommand] ❌ Grid data size mismatch! Expected " 
                  << expectedSize << ", got " << data.size() << std::endl;
        std::cerr << "[SculptCommand] Undo/Redo may not work correctly with new sparse format!" << std::endl;
        
        // Fallback: try to restore anyway (may be corrupted)
        grid->fromDenseArray(data);
    } else {
        std::cerr << "[SculptCommand] ❌ Empty data array, cannot restore!" << std::endl;
    }
    
    // Mark GPU as dirty so it will be re-uploaded
    grid->gpuDirty_ = true;
}

} // namespace Urbaxio::Engine

