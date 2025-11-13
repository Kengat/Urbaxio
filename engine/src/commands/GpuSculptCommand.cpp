#include "engine/commands/GpuSculptCommand.h"
#include "engine/scene.h"
#include "engine/scene_object.h"
#include "engine/geometry/VolumetricGeometry.h"
#include "engine/geometry/VoxelGrid.h"
#include "engine/gpu/GpuVoxelManager.h"
#include "engine/gpu/GpuMeshingKernels.h"
#include "engine/MeshManager.h"
#include <iostream>
#include <cuda_runtime.h>
#include <string>

namespace Urbaxio::Engine {

GpuSculptCommand::GpuSculptCommand(
    Scene* scene,
    GpuVoxelManager* gpuManager,
    uint64_t objectId,
    uint64_t* activeHandlePtr,
    uint64_t undoHandle,
    uint64_t redoHandle)
    : scene_(scene),
      gpuManager_(gpuManager),
      objectId_(objectId),
      activeHandlePtr_(activeHandlePtr),
      undoHandle_(undoHandle),
      redoHandle_(redoHandle)
{}

GpuSculptCommand::~GpuSculptCommand() {
    // Release GPU resources when command is destroyed
    // (only when command manager discards old undo history)
    if (undoHandle_ != 0 && undoHandle_ != *activeHandlePtr_) {
        gpuManager_->ReleaseGrid(undoHandle_);
    }
    if (redoHandle_ != 0 && redoHandle_ != *activeHandlePtr_) {
        gpuManager_->ReleaseGrid(redoHandle_);
    }
}

const char* GpuSculptCommand::GetName() const {
    return "GPU Sculpt Stroke";
}

void GpuSculptCommand::Execute() {
    // Redo: swap to redo handle
    std::cout << "[GpuSculptCommand] Executing (Redo)" << std::endl;
    swapToHandle(redoHandle_);
}

void GpuSculptCommand::Undo() {
    // Undo: swap to undo handle
    std::cout << "[GpuSculptCommand] Undoing" << std::endl;
    swapToHandle(undoHandle_);
}

void GpuSculptCommand::swapToHandle(uint64_t targetHandle) {
    if (!scene_ || !gpuManager_ || !activeHandlePtr_) return;
    
    SceneObject* obj = scene_->get_object_by_id(objectId_);
    if (!obj) return;
    
    auto* volGeom = dynamic_cast<VolumetricGeometry*>(obj->getGeometry());
    if (!volGeom) return;
    
    // Swap active handle (NO GPU data copy, just pointer swap!)
    *activeHandlePtr_ = targetHandle;
    
    std::cout << "[GpuSculptCommand] Swapped to handle " << targetHandle << std::endl;
    
    // Mark grid as dirty for next GPU operation
    volGeom->getGrid()->gpuDirty_ = true;
    
    // Remesh immediately using GPU
    remeshObject();
}

void GpuSculptCommand::remeshObject() {
    if (!scene_ || !gpuManager_ || !activeHandlePtr_) return;
    
    SceneObject* obj = scene_->get_object_by_id(objectId_);
    if (!obj) return;
    
    auto* volGeom = dynamic_cast<VolumetricGeometry*>(obj->getGeometry());
    if (!volGeom) return;
    
#ifdef URBAXIO_GPU_ENABLED
#if URBAXIO_GPU_ENABLED
    // Get GPU mesh buffers
    uint64_t leafCount = gpuManager_->GetLeafCount(*activeHandlePtr_);
    
    float* d_vertices = nullptr;
    float* d_normals = nullptr;
    int triangleCount = 0;
    
    bool success = GpuMeshingKernels::MarchingCubesAsync(
        gpuManager_->GetDeviceGridPointer(*activeHandlePtr_),
        leafCount,
        0.0f,
        volGeom->getGrid()->voxelSize,
        &d_vertices,
        &d_normals,
        &triangleCount,
        nullptr  // Default stream
    );
    
    if (success && triangleCount > 0) {
        cudaDeviceSynchronize();
        
        // D2D mesh update
        std::string meshId = std::to_string(objectId_);
        scene_->getMeshManager()->UpdateMeshFromGpuBuffers(
            meshId,
            d_vertices,
            d_normals,
            triangleCount * 3,
            nullptr
        );
        
        scene_->MarkStaticGeometryDirty();
        
        cudaFree(d_vertices);
        cudaFree(d_normals);
        
        std::cout << "[GpuSculptCommand] ✅ Remeshed: " << triangleCount << " tris" << std::endl;
    }
#endif
#endif
}

} // namespace Urbaxio::Engine


