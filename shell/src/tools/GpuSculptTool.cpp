#include "tools/GpuSculptTool.h"

#ifdef URBAXIO_GPU_ENABLED
#if URBAXIO_GPU_ENABLED
#include "engine/gpu/GpuVoxelManager.h"
#include "engine/gpu/GpuSculptKernels.h"
#include "engine/gpu/GpuMeshingKernels.h"
#include "engine/gpu/GpuVoxelConverter.h"
#endif
#endif

#include "engine/geometry/VoxelGrid.h"
#include "engine/Scene.h"
#include "engine/scene_object.h"
#include "engine/geometry/VolumetricGeometry.h"
#include "engine/commands/SculptCommand.h"
#include "engine/commands/CommandManager.h"
#include "LoadingManager.h"
#include "imgui.h"
#include <glm/glm.hpp>
#include <iostream>

namespace Urbaxio::Shell {

struct GpuSculptTool::Impl {
    Tools::ToolContext context;
    Engine::GpuVoxelManager* gpuManager = nullptr;
    
    // Tool state
    bool isSculpting = false;
    uint64_t activeObjectId = 0;
    glm::vec3 lastHitPoint{0.0f};
    
    // Brush settings
    float brushRadius = 0.5f;
    float brushStrength = 1.0f;
    int brushMode = 0; // 0=add, 1=subtract
    
    // Captured state for undo
    std::unique_ptr<Engine::VoxelGrid> capturedGrid;
};

GpuSculptTool::GpuSculptTool() : impl_(std::make_unique<Impl>()) {
#ifdef URBAXIO_GPU_ENABLED
#if URBAXIO_GPU_ENABLED
    impl_->gpuManager = new Engine::GpuVoxelManager();
    if (Engine::GpuVoxelManager::IsGpuAvailable()) {
        std::cout << "[GpuSculptTool] GPU manager initialized" << std::endl;
    } else {
        std::cout << "[GpuSculptTool] WARNING: GPU not available, tool will not work!" << std::endl;
    }
#endif
#endif
}

GpuSculptTool::~GpuSculptTool() {
#ifdef URBAXIO_GPU_ENABLED
#if URBAXIO_GPU_ENABLED
    delete impl_->gpuManager;
#endif
#endif
}

void GpuSculptTool::Activate(const Tools::ToolContext& context) {
    Tools::ITool::Activate(context);
    impl_->context = context;
    std::cout << "[GpuSculptTool] Tool activated" << std::endl;
}

void GpuSculptTool::Deactivate() {
    Tools::ITool::Deactivate();
    impl_->isSculpting = false;
    impl_->activeObjectId = 0;
}

void GpuSculptTool::OnLeftMouseDown(int mouseX, int mouseY, bool shift, bool ctrl, const glm::vec3& rayOrigin, const glm::vec3& rayDir) {
    if (!impl_->context.scene) return;
    
#ifdef URBAXIO_GPU_ENABLED
#if URBAXIO_GPU_ENABLED
    if (!impl_->gpuManager || !Engine::GpuVoxelManager::IsGpuAvailable()) {
        std::cout << "[GpuSculptTool] GPU not available" << std::endl;
        return;
    }
#else
    std::cout << "[GpuSculptTool] GPU support not compiled" << std::endl;
    return;
#endif
#else
    std::cout << "[GpuSculptTool] GPU support not compiled" << std::endl;
    return;
#endif

    // Snap to find target object
    SnappingSystem snapper;
    SnapResult snap = snapper.FindSnapPointFromRay(rayOrigin, rayDir, *impl_->context.scene);
    if (snap.snappedEntityId == 0) return;

    auto* obj = impl_->context.scene->get_object_by_id(snap.snappedEntityId);
    if (!obj) return;

    auto* volGeo = dynamic_cast<Engine::VolumetricGeometry*>(obj->getGeometry());
    if (!volGeo) {
        std::cout << "[GpuSculptTool] Selected object is not volumetric" << std::endl;
        return;
    }

    // Capture initial state for undo
    // Create a deep copy using OpenVDB's deepCopy method
    impl_->capturedGrid = std::make_unique<Engine::VoxelGrid>(
        volGeo->getGrid()->dimensions,
        volGeo->getGrid()->origin,
        volGeo->getGrid()->voxelSize
    );
    impl_->capturedGrid->grid_ = volGeo->getGrid()->grid_->deepCopy();
    impl_->capturedGrid->backgroundValue_ = volGeo->getGrid()->backgroundValue_;
    
    impl_->activeObjectId = snap.snappedEntityId;
    impl_->lastHitPoint = snap.worldPoint;
    impl_->isSculpting = true;

    std::cout << "[GpuSculptTool] Started sculpting object " << snap.snappedEntityId << std::endl;
}

void GpuSculptTool::OnLeftMouseUp(int mouseX, int mouseY, bool shift, bool ctrl) {
    if (!impl_->isSculpting) return;
    
    auto* obj = impl_->context.scene->get_object_by_id(impl_->activeObjectId);
    
    // Request async remesh NOW (once at the end, like CPU tool)
    if (obj && impl_->context.loadingManager) {
        impl_->context.loadingManager->RequestRemesh(impl_->context.scene, impl_->activeObjectId, false);
        obj->markMeshAsClean(); // Prevents synchronous remesh
        std::cout << "[GpuSculptTool] Async remesh requested (batched)" << std::endl;
    }
    
    // Create undo command
    if (obj && impl_->capturedGrid) {
        auto* volGeo = dynamic_cast<Engine::VolumetricGeometry*>(obj->getGeometry());
        if (volGeo) {
            // Convert grids to dense arrays for SculptCommand
            auto oldData = impl_->capturedGrid->toDenseArray();
            auto newData = volGeo->getGrid()->toDenseArray();
            
            auto cmd = std::make_unique<Engine::SculptCommand>(
                impl_->context.scene,
                impl_->activeObjectId,
                std::move(oldData),
                std::move(newData)
            );
            
            impl_->context.scene->getCommandManager()->ExecuteCommand(std::move(cmd));
        }
    }
    
    impl_->isSculpting = false;
    impl_->activeObjectId = 0;
    impl_->capturedGrid.reset();
    
    std::cout << "[GpuSculptTool] Sculpt stroke completed" << std::endl;
}

void GpuSculptTool::OnMouseMove(int mouseX, int mouseY) {
    // Movement handled in OnUpdate
}

void GpuSculptTool::OnUpdate(const SnapResult& snap, const glm::vec3& rayOrigin, const glm::vec3& rayDirection) {
    if (!impl_->isSculpting || impl_->activeObjectId == 0) return;
    
#ifdef URBAXIO_GPU_ENABLED
#if URBAXIO_GPU_ENABLED
    if (!impl_->gpuManager || !Engine::GpuVoxelManager::IsGpuAvailable()) return;
    
    auto* obj = impl_->context.scene->get_object_by_id(impl_->activeObjectId);
    if (!obj) return;
    
    auto* volGeo = dynamic_cast<Engine::VolumetricGeometry*>(obj->getGeometry());
    if (!volGeo) return;
    
    // Check if we're still over the same object
    if (snap.snappedEntityId != impl_->activeObjectId) return;
    
    impl_->lastHitPoint = snap.worldPoint;
    
    // Upload grid to GPU if needed
    auto* grid = volGeo->getGrid();
    if (grid->gpuHandle_ == 0 || grid->gpuDirty_) {
        grid->gpuHandle_ = impl_->gpuManager->UploadGrid(grid);
        grid->gpuDirty_ = false;
        std::cout << "[GpuSculptTool] Grid uploaded to GPU (handle: " << grid->gpuHandle_ << ")" << std::endl;
    }
    
    // Apply GPU sculpting
    void* devicePtr = impl_->gpuManager->GetDeviceGridPointer(grid->gpuHandle_);
    if (!devicePtr) {
        std::cout << "[GpuSculptTool] Failed to get device pointer" << std::endl;
        return;
    }
    
    Engine::GpuSculptKernels::SculptMode mode = impl_->brushMode == 0 ?
        Engine::GpuSculptKernels::SculptMode::ADD :
        Engine::GpuSculptKernels::SculptMode::SUBTRACT;
    
    // CRITICAL: Convert brush world position to index space (like CPU SculptTool does)
    openvdb::Vec3d brushIndexPos = grid->grid_->transform().worldToIndex(
        openvdb::Vec3d(snap.worldPoint.x, snap.worldPoint.y, snap.worldPoint.z)
    );
    glm::vec3 brushPosIndexSpace(brushIndexPos.x(), brushIndexPos.y(), brushIndexPos.z());
    
    // Convert brush radius from world units to voxel/index units
    float brushRadiusInVoxels = impl_->brushRadius / grid->voxelSize;
    
    // Get current grid bounding box (for reference, NOT for clamping)
    auto bbox = grid->getActiveBounds();
    glm::ivec3 gridBBoxMin(bbox.min().x(), bbox.min().y(), bbox.min().z());
    glm::ivec3 gridBBoxMax(bbox.max().x(), bbox.max().y(), bbox.max().z());
    
    // Prepare output buffers for GPU sculpting results
    std::vector<float> modifiedBuffer;
    glm::ivec3 minVoxel, maxVoxel;
    
    bool success = Engine::GpuSculptKernels::ApplySphericalBrush(
        devicePtr,
        brushPosIndexSpace,     // ✅ Now in INDEX SPACE (matches CPU tool)
        brushRadiusInVoxels,    // ✅ Now in VOXEL UNITS (matches CPU tool)
        1.0f,                   // voxelSize = 1.0 in index space
        mode,
        impl_->brushStrength,
        gridBBoxMin,            // Grid bbox from CPU (for expansion, not clamping)
        gridBBoxMax,            // Grid bbox from CPU (for expansion, not clamping)
        &modifiedBuffer,        // Output: modified dense buffer
        &minVoxel,              // Output: min coordinate
        &maxVoxel               // Output: max coordinate
    );
    
    if (success && !modifiedBuffer.empty()) {
        std::cout << "[GpuSculptTool] ✅ GPU sculpting succeeded, applying to OpenVDB grid..." << std::endl;
        
        // Apply GPU-modified buffer back to OpenVDB grid (CPU-side)
        bool applied = Engine::ApplyModifiedRegionToVoxelGrid(
            grid,
            modifiedBuffer,
            minVoxel,
            maxVoxel
        );
        
        if (applied) {
            // Update the object's AABB immediately after modification (like CPU SculptTool)
            openvdb::CoordBBox bbox = grid->grid_->evalActiveVoxelBoundingBox();
            if (!bbox.empty()) {
                openvdb::Vec3d worldMin = grid->grid_->indexToWorld(bbox.min());
                openvdb::Vec3d worldMax = grid->grid_->indexToWorld(bbox.max());
                obj->aabbMin = glm::vec3(worldMin.x(), worldMin.y(), worldMin.z());
                obj->aabbMax = glm::vec3(worldMax.x(), worldMax.y(), worldMax.z());
                obj->aabbValid = true;
            } else {
                obj->aabbValid = false; // Grid is empty
            }
            
            // Mark grid as dirty (needs re-upload for next frame)
            grid->gpuDirty_ = true;
            
            // NOTE: Don't remesh here! Wait for OnMouseUp (batching like CPU tool)
            // Remeshing every frame is expensive and causes lag
            
            // std::cout << "[GpuSculptTool] ✅ GPU sculpting complete (mesh update deferred)" << std::endl;
        } else {
            std::cerr << "[GpuSculptTool] ❌ Failed to apply modified region!" << std::endl;
        }
    } else {
        std::cerr << "[GpuSculptTool] ❌ GPU sculpting failed or returned empty buffer" << std::endl;
    }
#endif
#endif
}

void GpuSculptTool::RenderUI() {
    ImGui::Text("GPU Sculpt Tool (EXPERIMENTAL)");
    ImGui::Separator();
    
#ifdef URBAXIO_GPU_ENABLED
#if URBAXIO_GPU_ENABLED
    if (impl_->gpuManager && Engine::GpuVoxelManager::IsGpuAvailable()) {
        ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "GPU Available");
        
        // Show GPU memory usage
        auto stats = impl_->gpuManager->GetMemoryStats();
        ImGui::Text("GPU Grids: %zu (%.1f MB allocated)", stats.gridCount, stats.totalAllocated / 1024.0f / 1024.0f);
        ImGui::Text("GPU Memory: %.1f MB free / %.1f MB total", stats.freeGpuMemory / 1024.0f / 1024.0f, stats.totalGpuMemory / 1024.0f / 1024.0f);
        
        ImGui::Separator();
        ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "Status: Infrastructure Ready");
        ImGui::TextWrapped("GPU upload/download works. Full acceleration coming soon.");
        ImGui::TextDisabled("(Currently using hybrid CPU/GPU mode)");
    } else {
        ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "GPU Not Available");
    }
#else
    ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f), "GPU Support Disabled");
#endif
#else
    ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f), "GPU Support Disabled");
#endif
    
    ImGui::Separator();
    ImGui::SliderFloat("Brush Radius", &impl_->brushRadius, 0.05f, 10.0f);
    ImGui::SliderFloat("Brush Strength", &impl_->brushStrength, 0.1f, 2.0f);
    
    const char* modes[] = { "Add", "Subtract" };
    ImGui::Combo("Brush Mode", &impl_->brushMode, modes, 2);
    
    if (impl_->isSculpting) {
        ImGui::Separator();
        ImGui::TextColored(ImVec4(0.0f, 1.0f, 1.0f, 1.0f), "Sculpting...");
        ImGui::Text("Hit: (%.2f, %.2f, %.2f)", 
            impl_->lastHitPoint.x, 
            impl_->lastHitPoint.y, 
            impl_->lastHitPoint.z);
    }
}

} // namespace Urbaxio::Shell

