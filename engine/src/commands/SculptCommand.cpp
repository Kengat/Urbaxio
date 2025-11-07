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
    
    if (grid->sdfData.size() == data.size()) {
        grid->sdfData = data;
        obj->invalidateMeshCache();
        scene_->MarkStaticGeometryDirty();
    } else {
        std::cerr << "SculptCommand Error: Grid data size mismatch!" << std::endl;
    }
}

} // namespace Urbaxio::Engine

