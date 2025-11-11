// GPU Sculpt Tool - Optimized full GPU pipeline
// Grid lives on GPU during sculpting session, minimal CPU roundtrips



#include "tools/GpuSculptTool.h"



#ifdef URBAXIO_GPU_ENABLED
#if URBAXIO_GPU_ENABLED
#include "engine/gpu/GpuVoxelManager.h"
#include "engine/gpu/GpuSculptKernels.h"
#include "engine/gpu/GpuVoxelConverter.h"
#include <cuda_runtime.h>
#endif
#endif



#include "engine/geometry/VoxelGrid.h"
#include "engine/Scene.h"
#include "engine/scene_object.h"
#include "engine/geometry/VolumetricGeometry.h"
#include "engine/commands/SculptCommand.h"
#include "engine/commands/CommandManager.h"
#include "LoadingManager.h"
#include "snapping.h"
#include "camera.h"
#include "imgui.h"
#include <glm/glm.hpp>
#include <SDL2/SDL_mouse.h>
#include <iostream>
#include <limits>



namespace Urbaxio::Shell {



struct GpuSculptTool::Impl {
    Tools::ToolContext context;
    Engine::GpuVoxelManager* gpuManager = nullptr;
    
    // Sculpting state
    bool isSculpting = false;
    uint64_t activeObjectId = 0;
    glm::vec3 lastHitPoint{0.0f};
    glm::vec3 lastBrushApplyPos{0.0f};
    
    // Brush settings
    float brushRadius = 0.5f;
    float brushStrength = 1.0f;
    int brushMode = 0; // 0=add, 1=subtract
    float minBrushDistanceFactor = 0.15f; // Distance threshold for applying brush
    
    // GPU state (persistent during sculpting session)
    bool gridIsOnGpu = false;
    uint64_t currentGpuHandle = 0;
    
    // CUDA stream for async operations
    void* cudaStream = nullptr; // cudaStream_t as void*
    bool streamCreated = false;
    
    // Batch multiple brush strokes
    std::vector<glm::vec3> pendingBrushStrokes;
    static constexpr int MAX_BATCH_SIZE = 8;
    
    // Undo state
    std::unique_ptr<Engine::VoxelGrid> capturedGridCopy;
    
    // Performance tracking
    int strokeCount = 0;
    float totalGpuTime = 0.0f;
    
    // Adaptive distance threshold
    float getMinBrushDistance() const {
        // Smaller brushes need finer control
        if (brushRadius < 0.5f) return brushRadius * 0.05f;
        if (brushRadius < 1.0f) return brushRadius * 0.10f;
        return brushRadius * 0.15f;
    }
};



GpuSculptTool::GpuSculptTool() : impl_(std::make_unique<Impl>()) {
#ifdef URBAXIO_GPU_ENABLED
#if URBAXIO_GPU_ENABLED
    impl_->gpuManager = new Engine::GpuVoxelManager();
    if (Engine::GpuVoxelManager::IsGpuAvailable()) {
        std::cout << "[GpuSculptTool] ✅ GPU available and initialized" << std::endl;
        
        // Create dedicated CUDA stream for async sculpting
        cudaStream_t stream;
        if (cudaStreamCreate(&stream) == cudaSuccess) {
            impl_->cudaStream = stream;
            impl_->streamCreated = true;
            std::cout << "[GpuSculptTool] ✅ Created async CUDA stream" << std::endl;
        } else {
            std::cerr << "[GpuSculptTool] ⚠️ Failed to create CUDA stream" << std::endl;
        }
    } else {
        std::cout << "[GpuSculptTool] ❌ GPU not available!" << std::endl;
    }
#endif
#endif
}



GpuSculptTool::~GpuSculptTool() {
#ifdef URBAXIO_GPU_ENABLED
#if URBAXIO_GPU_ENABLED
    // Clean up CUDA stream
    if (impl_->streamCreated && impl_->cudaStream) {
        cudaStreamDestroy(static_cast<cudaStream_t>(impl_->cudaStream));
    }
    
    // Clean up GPU resources
    if (impl_->gridIsOnGpu && impl_->currentGpuHandle != 0) {
        impl_->gpuManager->ReleaseGrid(impl_->currentGpuHandle);
    }
    delete impl_->gpuManager;
#endif
#endif
}



void GpuSculptTool::Activate(const Tools::ToolContext& context) {
    Tools::ITool::Activate(context);
    impl_->context = context;
    impl_->strokeCount = 0;
    impl_->totalGpuTime = 0.0f;
    std::cout << "[GpuSculptTool] Tool activated" << std::endl;
}



void GpuSculptTool::Deactivate() {
    Tools::ITool::Deactivate();
    
    // Clean up GPU resources when deactivating
    if (impl_->gridIsOnGpu && impl_->currentGpuHandle != 0) {
#ifdef URBAXIO_GPU_ENABLED
#if URBAXIO_GPU_ENABLED
        impl_->gpuManager->ReleaseGrid(impl_->currentGpuHandle);
#endif
#endif
        impl_->gridIsOnGpu = false;
        impl_->currentGpuHandle = 0;
    }
    
    impl_->isSculpting = false;
    impl_->activeObjectId = 0;
    
    if (impl_->strokeCount > 0) {
        float avgTime = impl_->totalGpuTime / impl_->strokeCount;
        std::cout << "[GpuSculptTool] Session stats: " << impl_->strokeCount 
                  << " strokes, avg " << avgTime << "ms per stroke" << std::endl;
    }
}



void GpuSculptTool::OnLeftMouseDown(
    int mouseX, int mouseY, 
    bool shift, bool ctrl, 
    const glm::vec3& rayOrigin, 
    const glm::vec3& rayDir) 
{
    if (!impl_->context.scene) return;
    
#ifdef URBAXIO_GPU_ENABLED
#if URBAXIO_GPU_ENABLED
    if (!impl_->gpuManager || !Engine::GpuVoxelManager::IsGpuAvailable()) {
        std::cout << "[GpuSculptTool] ❌ GPU not available" << std::endl;
        return;
    }
#else
    std::cout << "[GpuSculptTool] ❌ GPU support not compiled" << std::endl;
    return;
#endif
#else
    std::cout << "[GpuSculptTool] ❌ GPU support not compiled" << std::endl;
    return;
#endif



    // Find target object using ray-mesh intersection
    glm::vec3 currentRayOrigin, currentRayDir;
    if (impl_->context.isVrMode) {
        currentRayOrigin = rayOrigin;
        currentRayDir = rayDir;
    } else {
        Camera::ScreenToWorldRay(
            mouseX, mouseY, 
            *impl_->context.display_w, *impl_->context.display_h, 
            impl_->context.camera->GetViewMatrix(), 
            impl_->context.camera->GetProjectionMatrix(
                (float)*impl_->context.display_w / (float)*impl_->context.display_h
            ), 
            currentRayOrigin, currentRayDir
        );
    }
    
    // Ray-mesh intersection to find precise hit point
    float closestHitDist = std::numeric_limits<float>::max();
    uint64_t hitObjectId = 0;
    glm::vec3 hitPosition;
    bool hit = false;



    for (auto* obj : impl_->context.scene->get_all_objects()) {
        if (!obj) continue;
        
        auto* volGeo = dynamic_cast<Engine::VolumetricGeometry*>(obj->getGeometry());
        if (!volGeo || !obj->hasMesh()) continue;
        
        // Broad phase: AABB check
        if (!obj->aabbValid) continue;
        
        // Simple ray-AABB intersection
        glm::vec3 invDir = 1.0f / currentRayDir;
        glm::vec3 t0s = (obj->aabbMin - currentRayOrigin) * invDir;
        glm::vec3 t1s = (obj->aabbMax - currentRayOrigin) * invDir;
        glm::vec3 tmin = glm::min(t0s, t1s);
        glm::vec3 tmax = glm::max(t0s, t1s);
        float t_enter = std::max({tmin.x, tmin.y, tmin.z});
        float t_exit = std::min({tmax.x, tmax.y, tmax.z});
        
        if (t_enter > t_exit || t_exit < 0) continue;
        if (t_enter > closestHitDist) continue;



        // Narrow phase: triangle intersection
        const auto& mesh = obj->getMeshBuffers();
        for (size_t i = 0; i < mesh.indices.size(); i += 3) {
            unsigned int i0 = mesh.indices[i];
            unsigned int i1 = mesh.indices[i+1];
            unsigned int i2 = mesh.indices[i+2];
            
            glm::vec3 v0(mesh.vertices[i0*3], mesh.vertices[i0*3+1], mesh.vertices[i0*3+2]);
            glm::vec3 v1(mesh.vertices[i1*3], mesh.vertices[i1*3+1], mesh.vertices[i1*3+2]);
            glm::vec3 v2(mesh.vertices[i2*3], mesh.vertices[i2*3+1], mesh.vertices[i2*3+2]);
            
            float t_tri;
            if (SnappingSystem::RayTriangleIntersect(currentRayOrigin, currentRayDir, v0, v1, v2, t_tri)) {
                if (t_tri > 0 && t_tri < closestHitDist) {
                    closestHitDist = t_tri;
                    hitObjectId = obj->get_id();
                    hit = true;
                }
            }
        }
    }



    if (!hit || hitObjectId == 0) return;
    
    hitPosition = currentRayOrigin + currentRayDir * closestHitDist;
    
    // Get object and validate
    auto* obj = impl_->context.scene->get_object_by_id(hitObjectId);
    if (!obj) return;



    auto* volGeo = dynamic_cast<Engine::VolumetricGeometry*>(obj->getGeometry());
    if (!volGeo) {
        std::cout << "[GpuSculptTool] ❌ Selected object is not volumetric" << std::endl;
        return;
    }



    // Start sculpting session
    impl_->activeObjectId = hitObjectId;
    impl_->lastHitPoint = hitPosition;
    impl_->lastBrushApplyPos = hitPosition;
    impl_->isSculpting = true;



    // Capture state for undo (deep copy)
    impl_->capturedGridCopy = std::make_unique<Engine::VoxelGrid>(
        volGeo->getGrid()->dimensions,
        volGeo->getGrid()->origin,
        volGeo->getGrid()->voxelSize
    );
    impl_->capturedGridCopy->grid_ = volGeo->getGrid()->grid_->deepCopy();
    impl_->capturedGridCopy->backgroundValue_ = volGeo->getGrid()->backgroundValue_;
    
#ifdef URBAXIO_GPU_ENABLED
#if URBAXIO_GPU_ENABLED
    // Upload grid to GPU (if not already there)
    auto* grid = volGeo->getGrid();
    if (!impl_->gridIsOnGpu || impl_->currentGpuHandle == 0 || grid->gpuDirty_) {
        // Release old handle if exists
        if (impl_->currentGpuHandle != 0) {
            impl_->gpuManager->ReleaseGrid(impl_->currentGpuHandle);
        }
        
        // Upload new grid
        impl_->currentGpuHandle = impl_->gpuManager->UploadGrid(grid);
        impl_->gridIsOnGpu = (impl_->currentGpuHandle != 0);
        grid->gpuDirty_ = false;
        
        if (impl_->gridIsOnGpu) {
            std::cout << "[GpuSculptTool] ✅ Grid uploaded to GPU (handle: " 
                      << impl_->currentGpuHandle << ")" << std::endl;
        } else {
            std::cout << "[GpuSculptTool] ❌ Failed to upload grid to GPU" << std::endl;
            impl_->isSculpting = false;
            return;
        }
    }
    
    // Apply first brush stroke
    applyBrushGPU(hitPosition);
#endif
#endif
    
    std::cout << "[GpuSculptTool] ✅ Started sculpting object " << hitObjectId << std::endl;
}



void GpuSculptTool::OnLeftMouseUp(int mouseX, int mouseY, bool shift, bool ctrl) {
    if (!impl_->isSculpting) return;
    
    auto* obj = impl_->context.scene->get_object_by_id(impl_->activeObjectId);
    if (!obj) {
        impl_->isSculpting = false;
        return;
    }
    
    auto* volGeo = dynamic_cast<Engine::VolumetricGeometry*>(obj->getGeometry());
    if (!volGeo) {
        impl_->isSculpting = false;
        return;
    }
    
#ifdef URBAXIO_GPU_ENABLED
#if URBAXIO_GPU_ENABLED
    // Apply any remaining batched strokes
    applyBrushBatchGPU();
    
    // Synchronize stream before download
    if (impl_->streamCreated && impl_->cudaStream) {
        cudaStreamSynchronize(static_cast<cudaStream_t>(impl_->cudaStream));
    }
    
    // Download modified grid from GPU back to CPU
    if (impl_->gridIsOnGpu && impl_->currentGpuHandle != 0) {
        bool downloaded = impl_->gpuManager->DownloadGrid(
            impl_->currentGpuHandle, 
            volGeo->getGrid()
        );
        
        if (downloaded) {
            std::cout << "[GpuSculptTool] ✅ Downloaded modified grid from GPU" << std::endl;
            
            // Update object AABB from modified grid
            openvdb::CoordBBox bbox = volGeo->getGrid()->grid_->evalActiveVoxelBoundingBox();
            if (!bbox.empty()) {
                openvdb::Vec3d worldMin = volGeo->getGrid()->grid_->indexToWorld(bbox.min());
                openvdb::Vec3d worldMax = volGeo->getGrid()->grid_->indexToWorld(bbox.max());
                obj->aabbMin = glm::vec3(worldMin.x(), worldMin.y(), worldMin.z());
                obj->aabbMax = glm::vec3(worldMax.x(), worldMax.y(), worldMax.z());
                obj->aabbValid = true;
            } else {
                obj->aabbValid = false;
            }
            
            // Mark for re-upload next time (topology may have changed on CPU side for undo)
            volGeo->getGrid()->gpuDirty_ = true;
        } else {
            std::cerr << "[GpuSculptTool] ❌ Failed to download grid from GPU" << std::endl;
        }
    }
#endif
#endif
    
    // Request async remesh (like CPU SculptTool)
    if (impl_->context.loadingManager) {
        impl_->context.loadingManager->RequestRemesh(
            impl_->context.scene, 
            impl_->activeObjectId, 
            false // Use CPU meshing (OpenVDB volumeToMesh - already optimized)
        );
        obj->markMeshAsClean(); // Prevents synchronous remesh
        std::cout << "[GpuSculptTool] ✅ Async remesh requested" << std::endl;
    }
    
    // Create undo command
    if (impl_->capturedGridCopy) {
        auto oldData = impl_->capturedGridCopy->toFullDenseArray();
        auto newData = volGeo->getGrid()->toFullDenseArray();
        
        auto cmd = std::make_unique<Engine::SculptCommand>(
            impl_->context.scene,
            impl_->activeObjectId,
            std::move(oldData),
            std::move(newData)
        );
        
        impl_->context.scene->getCommandManager()->ExecuteCommand(std::move(cmd));
    }
    
    // Clean up session state
    impl_->isSculpting = false;
    impl_->activeObjectId = 0;
    impl_->capturedGridCopy.reset();
    
    std::cout << "[GpuSculptTool] ✅ Sculpt stroke completed" << std::endl;
}



void GpuSculptTool::OnMouseMove(int mouseX, int mouseY) {
    // Handled in OnUpdate
}



void GpuSculptTool::OnUpdate(
    const SnapResult& snap, 
    const glm::vec3& rayOrigin, 
    const glm::vec3& rayDirection) 
{
    if (!impl_->isSculpting || impl_->activeObjectId == 0) return;
    
#ifdef URBAXIO_GPU_ENABLED
#if URBAXIO_GPU_ENABLED
    if (!impl_->gpuManager || !Engine::GpuVoxelManager::IsGpuAvailable()) return;
    if (!impl_->gridIsOnGpu || impl_->currentGpuHandle == 0) return;
    
    auto* obj = impl_->context.scene->get_object_by_id(impl_->activeObjectId);
    if (!obj) return;
    
    auto* volGeo = dynamic_cast<Engine::VolumetricGeometry*>(obj->getGeometry());
    if (!volGeo) return;
    
    // Get current ray and find hit point
    glm::vec3 currentRayOrigin, currentRayDir;
    if (impl_->context.isVrMode) {
        currentRayOrigin = rayOrigin;
        currentRayDir = rayDirection;
    } else {
        int mouseX, mouseY;
        SDL_GetMouseState(&mouseX, &mouseY);
        Camera::ScreenToWorldRay(
            mouseX, mouseY, 
            *impl_->context.display_w, *impl_->context.display_h, 
            impl_->context.camera->GetViewMatrix(), 
            impl_->context.camera->GetProjectionMatrix(
                (float)*impl_->context.display_w / (float)*impl_->context.display_h
            ), 
            currentRayOrigin, currentRayDir
        );
    }
    
    // Find hit point on mesh
    float closestHitDist = std::numeric_limits<float>::max();
    bool hit = false;
    
    const auto& mesh = obj->getMeshBuffers();
    for (size_t i = 0; i < mesh.indices.size(); i += 3) {
        unsigned int i0 = mesh.indices[i];
        unsigned int i1 = mesh.indices[i+1];
        unsigned int i2 = mesh.indices[i+2];
        
        glm::vec3 v0(mesh.vertices[i0*3], mesh.vertices[i0*3+1], mesh.vertices[i0*3+2]);
        glm::vec3 v1(mesh.vertices[i1*3], mesh.vertices[i1*3+1], mesh.vertices[i1*3+2]);
        glm::vec3 v2(mesh.vertices[i2*3], mesh.vertices[i2*3+1], mesh.vertices[i2*3+2]);
        
        float t_tri;
        if (SnappingSystem::RayTriangleIntersect(currentRayOrigin, currentRayDir, v0, v1, v2, t_tri)) {
            if (t_tri > 0 && t_tri < closestHitDist) {
                closestHitDist = t_tri;
                hit = true;
            }
        }
    }
    
    if (!hit) return;
    
    glm::vec3 hitPosition = currentRayOrigin + currentRayDir * closestHitDist;
    impl_->lastHitPoint = hitPosition;
    
    // Distance threshold to avoid over-applying brush (adaptive)
    float minDist = impl_->getMinBrushDistance();
    if (glm::distance(hitPosition, impl_->lastBrushApplyPos) >= minDist) {
        // Add to batch instead of immediate apply
        impl_->pendingBrushStrokes.push_back(hitPosition);
        
        // Apply batch when accumulated enough strokes
        if (impl_->pendingBrushStrokes.size() >= impl_->MAX_BATCH_SIZE) {
            applyBrushBatchGPU();
        }
        
        impl_->lastBrushApplyPos = hitPosition;
    }
#endif
#endif
}



// Apply multiple brush strokes in one GPU call
bool GpuSculptTool::applyBrushBatchGPU() {
    if (impl_->pendingBrushStrokes.empty()) return false;
    
    for (const auto& pos : impl_->pendingBrushStrokes) {
        applyBrushGPU(pos); // These will queue up in the async stream
    }
    
    impl_->pendingBrushStrokes.clear();
    return true;
}

bool GpuSculptTool::applyBrushGPU(const glm::vec3& brushWorldPos) {
#ifdef URBAXIO_GPU_ENABLED
#if URBAXIO_GPU_ENABLED
    if (!impl_->gridIsOnGpu || impl_->currentGpuHandle == 0) return false;
    
    auto* obj = impl_->context.scene->get_object_by_id(impl_->activeObjectId);
    if (!obj) return false;
    
    auto* volGeo = dynamic_cast<Engine::VolumetricGeometry*>(obj->getGeometry());
    if (!volGeo) return false;
    
    auto* grid = volGeo->getGrid();
    
    // Get device pointer
    void* devicePtr = impl_->gpuManager->GetDeviceGridPointer(impl_->currentGpuHandle);
    if (!devicePtr) {
        std::cerr << "[GpuSculptTool] ❌ Failed to get device pointer" << std::endl;
        return false;
    }
    
    // Get leaf count from manager (stored on CPU to avoid GPU access)
    uint64_t leafCount = impl_->gpuManager->GetLeafCount(impl_->currentGpuHandle);
    if (leafCount == 0) {
        std::cerr << "[GpuSculptTool] ❌ Grid has no leaves!" << std::endl;
        return false;
    }
    
    // Convert brush position from world to index space
    openvdb::Vec3d brushIndexPos = grid->grid_->transform().worldToIndex(
        openvdb::Vec3d(brushWorldPos.x, brushWorldPos.y, brushWorldPos.z)
    );
    glm::vec3 brushPosIndexSpace(brushIndexPos.x(), brushIndexPos.y(), brushIndexPos.z());
    
    // Convert brush radius to voxel units
    float brushRadiusInVoxels = impl_->brushRadius / grid->voxelSize;
    
    // Get grid bounds (for reference)
    auto bbox = grid->getActiveBounds();
    glm::ivec3 gridBBoxMin(bbox.min().x(), bbox.min().y(), bbox.min().z());
    glm::ivec3 gridBBoxMax(bbox.max().x(), bbox.max().y(), bbox.max().z());
    
    // Apply GPU sculpting (direct modification on GPU)
    Engine::GpuSculptKernels::SculptMode mode = 
        (impl_->brushMode == 0) ? 
        Engine::GpuSculptKernels::SculptMode::ADD : 
        Engine::GpuSculptKernels::SculptMode::SUBTRACT;
    
    auto t_start = std::chrono::high_resolution_clock::now();
    
    bool success = Engine::GpuSculptKernels::ApplySphericalBrush(
        devicePtr,
        leafCount,              // Pass leaf count (obtained on CPU)
        brushPosIndexSpace,
        brushRadiusInVoxels,
        1.0f,                    // voxelSize (not used in index space)
        mode,
        impl_->brushStrength,
        gridBBoxMin,
        gridBBoxMax,
        nullptr, nullptr, nullptr, // No output buffers needed
        impl_->cudaStream         // Pass stream for async execution
    );
    
    auto t_end = std::chrono::high_resolution_clock::now();
    float elapsed = std::chrono::duration<float, std::milli>(t_end - t_start).count();
    
    if (success) {
        impl_->strokeCount++;
        impl_->totalGpuTime += elapsed;
        // std::cout << "[GpuSculptTool] ✅ GPU sculpt: " << elapsed << "ms" << std::endl;
    } else {
        std::cerr << "[GpuSculptTool] ❌ GPU sculpting failed" << std::endl;
    }
    
    return success;
#else
    return false;
#endif
#else
    return false;
#endif
}



void GpuSculptTool::RenderUI() {
    ImGui::Text("GPU Sculpt Tool");
    ImGui::Separator();
    
#ifdef URBAXIO_GPU_ENABLED
#if URBAXIO_GPU_ENABLED
    if (impl_->gpuManager && Engine::GpuVoxelManager::IsGpuAvailable()) {
        ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "GPU Available");
        
        // Memory stats
        auto stats = impl_->gpuManager->GetMemoryStats();
        ImGui::Text("GPU Grids: %zu (%.1f MB)", 
            stats.gridCount, 
            stats.totalAllocated / 1024.0f / 1024.0f);
        ImGui::Text("GPU Memory: %.1f MB free / %.1f MB total", 
            stats.freeGpuMemory / 1024.0f / 1024.0f, 
            stats.totalGpuMemory / 1024.0f / 1024.0f);
        
        // Session stats
        if (impl_->strokeCount > 0) {
            float avgTime = impl_->totalGpuTime / impl_->strokeCount;
            ImGui::Text("Session: %d strokes, avg %.2fms", impl_->strokeCount, avgTime);
        }
        
        ImGui::Separator();
        if (impl_->isSculpting) {
            ImGui::TextColored(ImVec4(0.0f, 1.0f, 1.0f, 1.0f), "Sculpting...");
            if (impl_->gridIsOnGpu) {
                ImGui::Text("Grid on GPU (handle: %llu)", impl_->currentGpuHandle);
            }
        } else {
            ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f), "Idle");
        }
    } else {
        ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "GPU Not Available");
    }
#else
    ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f), "GPU Support Disabled (Compile with CUDA)");
#endif
#else
    ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f), "GPU Support Disabled (Compile with CUDA)");
#endif
    
    ImGui::Separator();
    ImGui::SliderFloat("Brush Radius", &impl_->brushRadius, 0.05f, 5.0f);
    ImGui::SliderFloat("Brush Strength", &impl_->brushStrength, 0.1f, 2.0f);
    
    const char* modes[] = { "Add", "Subtract" };
    ImGui::Combo("Brush Mode", &impl_->brushMode, modes, 2);
    
    ImGui::Separator();
    ImGui::TextDisabled("Meshing: CPU (OpenVDB volumeToMesh)");
    ImGui::TextDisabled("Updates: Async (background thread)");
}



} // namespace Urbaxio::Shell
