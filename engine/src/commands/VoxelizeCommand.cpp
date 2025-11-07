#include "engine/commands/VoxelizeCommand.h"
#include "engine/scene.h"
#include "engine/scene_object.h"
#include "engine/geometry/BRepGeometry.h"
#include "engine/geometry/VolumetricGeometry.h"
#include "engine/geometry/VoxelGrid.h"
#include <iostream>

namespace Urbaxio::Engine {


VoxelizeCommand::VoxelizeCommand(Scene* scene, uint64_t objectId, std::unique_ptr<IGeometry> voxelizedGeometry)
    : scene_(scene), objectId_(objectId), voxelizedGeometry_(std::move(voxelizedGeometry)) {}
VoxelizeCommand::~VoxelizeCommand() = default;


const char* VoxelizeCommand::GetName() const {
    return "Voxelize Object";
}


void VoxelizeCommand::Execute() {
    if (!scene_) return;
    SceneObject* obj = scene_->get_object_by_id(objectId_);
    if (!obj || !voxelizedGeometry_) return;

    // If originalGeometry_ is null, it's the first execution.
    // Otherwise, it's a redo.
    if (!originalGeometry_) {
        originalGeometry_ = obj->setGeometry(std::move(voxelizedGeometry_));
    } else { // Redo

        originalGeometry_ = obj->setGeometry(std::move(voxelizedGeometry_));
        return;
    }

    // After swapping geometry, invalidate caches.
    obj->invalidateMeshCache();
    scene_->MarkStaticGeometryDirty();
}


void VoxelizeCommand::Undo() {
    if (!scene_ || !originalGeometry_) return;
    SceneObject* obj = scene_->get_object_by_id(objectId_);
    if (!obj) return;

    // Swap the original geometry back, and store the voxelized one for redo.
    voxelizedGeometry_ = obj->setGeometry(std::move(originalGeometry_));
    
    obj->invalidateMeshCache();
    scene_->MarkStaticGeometryDirty();
}


} // namespace Urbaxio::Engine

