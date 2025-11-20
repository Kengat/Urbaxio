// GPU Sculpt Tool - Optimized full GPU pipeline
// Grid lives on GPU during sculpting session, minimal CPU roundtrips



#include "tools/GpuSculptTool.h"



#ifdef URBAXIO_GPU_ENABLED
#if URBAXIO_GPU_ENABLED
#include "engine/gpu/GpuVoxelManager.h"
#include "engine/gpu/GpuSculptKernels.h"
#include "engine/gpu/GpuVoxelConverter.h"
#include "engine/gpu/GpuMeshingKernels.h"
#include "engine/gpu/GpuRaycastKernels.h"
#include "engine/gpu/GpuAabbKernels.h"
#include <cuda_runtime.h>
#endif
#endif



#include "engine/geometry/VoxelGrid.h"
#include "engine/Scene.h"
#include "engine/scene_object.h"
#include "engine/geometry/VolumetricGeometry.h"
#include "engine/commands/SculptCommand.h"
#include "engine/commands/GpuSculptCommand.h"
#include "engine/commands/CommandManager.h"
#include "engine/MeshManager.h"
#include "LoadingManager.h"
#include "snapping.h"
#include "renderer.h"
#include "camera.h"
#include "imgui.h"
#include <glm/glm.hpp>
#include <SDL2/SDL_mouse.h>
#include <iostream>
#include <limits>
#include <vector>
#include <openvdb/openvdb.h>
#include <chrono>
#include <functional>
#include <algorithm>
#include <string>



// После includes, перед namespace



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
    static constexpr int MAX_BATCH_SIZE = 64;
    
    uint64_t undoHandle = 0;
    
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
    
    // Clean up GPU resources when deactivating TOOL (not just stroke!)
    if (impl_->gridIsOnGpu && impl_->currentGpuHandle != 0) {
#ifdef URBAXIO_GPU_ENABLED
#if URBAXIO_GPU_ENABLED
        impl_->gpuManager->ReleaseGrid(impl_->currentGpuHandle);
        std::cout << "[GpuSculptTool] Cleaned up GPU grid (tool deactivated)" << std::endl;
#endif
#endif
        impl_->gridIsOnGpu = false;
        impl_->currentGpuHandle = 0;
    }
    
    impl_->isSculpting = false;
    impl_->activeObjectId = 0;
    
    if (impl_->context.renderer) {
        impl_->context.renderer->UpdateBrushPreview(glm::vec3(0), 0, glm::vec3(0), false);
    }
    
    if (impl_->strokeCount > 0) {
        float avgTime = impl_->totalGpuTime / impl_->strokeCount;
        std::cout << "[GpuSculptTool] Session stats: " << impl_->strokeCount 
                  << " strokes, avg " << avgTime << "ms per stroke" << std::endl;
    }
}

// --- NEW: Setter Implementations ---
void GpuSculptTool::SetBrushRadius(float radius) { 
    impl_->brushRadius = radius; 
}

void GpuSculptTool::SetBrushStrength(float strength) { 
    impl_->brushStrength = strength; 
}
// -----------------------------------

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
    
    // Быстрый AABB check для выбора объекта
    float closestAABBDist = std::numeric_limits<float>::max();
    Engine::SceneObject* hitObject = nullptr;
    
    for (auto* obj : impl_->context.scene->get_all_objects()) {
        if (!obj) continue;
        auto* volGeo = dynamic_cast<Engine::VolumetricGeometry*>(obj->getGeometry());
        if (!volGeo || !obj->hasMesh() || !obj->aabbValid) continue;
        
        glm::vec3 invDir = 1.0f / currentRayDir;
        glm::vec3 t0s = (obj->aabbMin - currentRayOrigin) * invDir;
        glm::vec3 t1s = (obj->aabbMax - currentRayOrigin) * invDir;
        glm::vec3 tmin = glm::min(t0s, t1s);
        glm::vec3 tmax = glm::max(t0s, t1s);
        float t_enter = std::max({tmin.x, tmin.y, tmin.z, 0.0f});
        float t_exit = std::min({tmax.x, tmax.y, tmax.z});
        
        if (t_enter <= t_exit && t_enter < closestAABBDist) {
            closestAABBDist = t_enter;
            hitObject = obj;
        }
    }
    
    if (!hitObject) return;
    
    // Step 2: Precise GPU raycast against live NanoVDB grid
    glm::vec3 hitPosition;
    bool hit = false;
    
    auto* obj = hitObject;
    if (!obj) return;

    auto* volGeo = dynamic_cast<Engine::VolumetricGeometry*>(obj->getGeometry());
    if (!volGeo) {
        std::cout << "[GpuSculptTool] ❌ Selected object is not volumetric" << std::endl;
        return;
    }

    // Upload grid to GPU first (if needed) for raycast
    auto* grid = volGeo->getGrid();

    // Check if we're switching to a different object
    bool switchingObjects = (impl_->activeObjectId != 0 && impl_->activeObjectId != hitObject->get_id());
    if (switchingObjects && impl_->currentGpuHandle != 0) {
        std::cout << "[GpuSculptTool] Switching objects: " << impl_->activeObjectId
                  << " -> " << hitObject->get_id() << std::endl;
        impl_->gpuManager->ReleaseGrid(impl_->currentGpuHandle);
        impl_->currentGpuHandle = 0;
        impl_->gridIsOnGpu = false;
    }

    if (!impl_->gridIsOnGpu || impl_->currentGpuHandle == 0 || grid->gpuDirty_) {
        if (impl_->currentGpuHandle != 0) {
            impl_->gpuManager->ReleaseGrid(impl_->currentGpuHandle);
        }
        impl_->currentGpuHandle = impl_->gpuManager->UploadGrid(grid);
        impl_->gridIsOnGpu = (impl_->currentGpuHandle != 0);
        grid->gpuDirty_ = false;
    }
    
    if (impl_->gridIsOnGpu && impl_->currentGpuHandle != 0) {
        // Raycast against existing GPU grid (if available)
        auto raycastResult = Engine::GpuRaycastKernels::RaycastNanoVDB(
            impl_->gpuManager->GetDeviceGridPointer(impl_->currentGpuHandle),
            currentRayOrigin,
            currentRayDir,
            1000.0f,  // maxDistance
            0.0f      // isoValue
        );
        
        if (raycastResult.hit) {
            hitPosition = raycastResult.hitPoint;
            hit = true;
        }
    }
    
    if (!hit) {
        // Fallback: use AABB intersection point
        hitPosition = currentRayOrigin + currentRayDir * closestAABBDist;
    }

    // Start sculpting session
    impl_->activeObjectId = hitObject->get_id();
    impl_->lastHitPoint = hitPosition;
    impl_->lastBrushApplyPos = hitPosition;
    impl_->isSculpting = true;

    // GPU-native Undo: Duplicate grid in VRAM (D2D copy, < 1ms!)
    impl_->undoHandle = impl_->gpuManager->DuplicateGrid(impl_->currentGpuHandle);
    std::cout << "[GpuSculptTool] Captured undo state (handle: "
              << impl_->undoHandle << ")" << std::endl;
    
#ifdef URBAXIO_GPU_ENABLED
#if URBAXIO_GPU_ENABLED
    if (impl_->gridIsOnGpu && impl_->currentGpuHandle != 0) {
        std::cout << "[GpuSculptTool] ✅ Grid already on GPU (handle: " 
                  << impl_->currentGpuHandle << ")" << std::endl;
    } else {
        std::cout << "[GpuSculptTool] ❌ Grid not on GPU!" << std::endl;
        impl_->isSculpting = false;
        return;
    }
    
    // DON'T mark grid as dirty - we're keeping it on GPU!
    
    // Apply first brush stroke
    std::cout << "[DEBUG] Applying initial brush at (" << hitPosition.x << ", " 
              << hitPosition.y << ", " << hitPosition.z << ")" << std::endl;
    bool success = applyBrushGPU(hitPosition);
    std::cout << "[DEBUG] Initial brush result: " << (success ? "SUCCESS" : "FAILED") << std::endl;
#endif
#endif
    
    std::cout << "[GpuSculptTool] ✅ Started sculpting object " << hitObject->get_id() << std::endl;
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
    
    // Perform final mesh update on GPU using D2D pipeline (no CPU copy!)
    uint64_t leafCount = impl_->gpuManager->GetLeafCount(impl_->currentGpuHandle);
    
    float* d_vertices = nullptr;
    float* d_normals = nullptr;
    int triangleCount = 0;
    
    bool meshSuccess = Engine::GpuMeshingKernels::MarchingCubesAsync(
        impl_->gpuManager->GetDeviceGridPointer(impl_->currentGpuHandle),
        leafCount,
        0.0f,  // isoValue
        volGeo->getGrid()->voxelSize,
        &d_vertices,  // Returns device pointer!
        &d_normals,   // Returns device pointer!
        &triangleCount,
        impl_->cudaStream
    );
    
    if (meshSuccess && triangleCount > 0) {
        // Wait for GPU meshing to complete
        cudaStreamSynchronize(static_cast<cudaStream_t>(impl_->cudaStream));
        
        // D2D update: VRAM -> VBO (no CPU copy!)
        std::string meshId = std::to_string(impl_->activeObjectId);
        auto* gpuMesh = impl_->context.meshManager->UpdateMeshFromGpuBuffers(
            meshId,
            d_vertices,
            d_normals,
            triangleCount * 3,  // vertexCount
            impl_->cudaStream
        );
        
        if (gpuMesh) {
            // Synchronize object rendering data with GPU mesh
            obj->setGpuManaged(true);
            obj->vao = gpuMesh->vao;
            obj->index_count = gpuMesh->index_count;

            // Update mesh groups for material batching
            obj->meshGroups.clear();
            Engine::MeshGroup defaultGroup;
            defaultGroup.materialName = "Default";
            defaultGroup.startIndex = 0;
            defaultGroup.indexCount = gpuMesh->index_count;
            obj->meshGroups.push_back(defaultGroup);

            // ✅ FIX: Compute AABB on GPU (only 24 bytes D2H!)
            glm::vec3 aabbMin, aabbMax;
            bool aabbSuccess = Engine::GpuAabbKernels::ComputeAabb(
                d_vertices,
                static_cast<size_t>(triangleCount) * 3,
                aabbMin,
                aabbMax,
                impl_->cudaStream
            );

            if (aabbSuccess) {
                obj->aabbMin = aabbMin;
                obj->aabbMax = aabbMax;
                obj->aabbValid = true;
                std::cout << "[GpuSculptTool] ✅ GPU AABB: min("
                          << aabbMin.x << "," << aabbMin.y << "," << aabbMin.z
                          << ") max(" << aabbMax.x << "," << aabbMax.y << "," 
                          << aabbMax.z << ")" << std::endl;
            } else {
                std::cerr << "[GpuSculptTool] ⚠️ GPU AABB computation failed" << std::endl;
                obj->aabbValid = false;
            }

            impl_->context.scene->MarkStaticGeometryDirty();
            std::cout << "[GpuSculptTool] ✅ D2D mesh update: " 
                  << triangleCount << " tris (GPU-managed)" << std::endl;
        }
    
        // Free temporary GPU buffers
        cudaFree(d_vertices);
        cudaFree(d_normals);
    }
    
    // GPU-native Undo: Create command with handles (no CPU copies!)
    if (impl_->undoHandle != 0) {
        // Duplicate current state for redo (D2D copy, < 1ms!)
        uint64_t redoHandle = impl_->gpuManager->DuplicateGrid(impl_->currentGpuHandle);
        
        auto cmd = std::make_unique<Engine::GpuSculptCommand>(
            impl_->context.scene,
            impl_->gpuManager,
            impl_->activeObjectId,
            &impl_->currentGpuHandle,  // Pointer - command can modify it!
            impl_->undoHandle,
            redoHandle
        );
        
        impl_->context.scene->getCommandManager()->PushCommand(std::move(cmd));
        
        std::cout << "[GpuSculptTool] ✅ GPU Undo created (no D2H copy!)" << std::endl;
        
        impl_->undoHandle = 0;  // Reset
    }
    
#endif
#endif
    
    impl_->isSculpting = false;
    
    // DON'T reset activeObjectId - keep session alive for next stroke!
    // impl_->activeObjectId = 0;  // ❌ This would lose the GPU grid
    
    std::cout << "[GpuSculptTool] ✅ Sculpt stroke completed (session active)" << std::endl;
}



void GpuSculptTool::OnMouseMove(int mouseX, int mouseY) {
    // Handled in OnUpdate
}



void GpuSculptTool::OnUpdate(
    const SnapResult& snap, 
    const glm::vec3& rayOrigin, 
    const glm::vec3& rayDirection) 
{
    if (!impl_->isSculpting) return;  // ✅ FIX: Removed meshCache check
    
#ifdef URBAXIO_GPU_ENABLED
#if URBAXIO_GPU_ENABLED
    if (!impl_->gridIsOnGpu || impl_->currentGpuHandle == 0) return;
    
    // Получить ray
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
    
    // GPU raycast against live NanoVDB grid (always up-to-date!)
    auto raycastResult = Engine::GpuRaycastKernels::RaycastNanoVDB(
        impl_->gpuManager->GetDeviceGridPointer(impl_->currentGpuHandle),
        currentRayOrigin,
        currentRayDir,
        1000.0f,
        0.0f
    );
    
    if (raycastResult.hit) {
        glm::vec3 hitPosition = raycastResult.hitPoint;
        
        if (impl_->context.renderer) {
            glm::vec3 brushColor = (impl_->brushMode == 0) 
                ? glm::vec3(0.2f, 0.8f, 1.0f)
                : glm::vec3(1.0f, 0.4f, 0.0f);
            impl_->context.renderer->UpdateBrushPreview(
                hitPosition,
                impl_->brushRadius,
                brushColor,
                true
            );
        }
        
        impl_->lastHitPoint = hitPosition;
        
        // Проверка дистанции для применения brush
        float minDist = impl_->getMinBrushDistance();
        if (glm::distance(hitPosition, impl_->lastBrushApplyPos) >= minDist) {
            impl_->pendingBrushStrokes.push_back(hitPosition);
            
            if (impl_->pendingBrushStrokes.size() >= impl_->MAX_BATCH_SIZE) {
                applyBrushBatchGPU();
            }
            
            impl_->lastBrushApplyPos = hitPosition;
        }
    } else {
        if (impl_->context.renderer) {
            impl_->context.renderer->UpdateBrushPreview(glm::vec3(0), 0, glm::vec3(0), false);
        }
        return;  // No surface hit
    }
    
    // TEMPORARY: Disabled async meshing due to deadlock
    // TODO: Fix GenerateTrianglesKernel with proper per-cell scan
    /*
        static int meshUpdateCounter = 0;
        if (++meshUpdateCounter >= 5) { // Update mesh every 5 brush strokes
            meshUpdateCounter = 0;
            
            auto* obj = impl_->context.scene->get_object_by_id(impl_->activeObjectId);
            if (obj) {
                auto* volGeo = dynamic_cast<Engine::VolumetricGeometry*>(obj->getGeometry());
                if (volGeo) {
                    // Get leaf count from manager
                    uint64_t leafCount = impl_->gpuManager->GetLeafCount(impl_->currentGpuHandle);
                    
                    float* d_vertices = nullptr;
                    float* d_normals = nullptr;
                    int triangleCount = 0;
                    
                    bool success = Engine::GpuMeshingKernels::MarchingCubesAsync(
                        impl_->gpuManager->GetDeviceGridPointer(impl_->currentGpuHandle),
                        leafCount,
                        0.0f,  // isoValue
                        volGeo->getGrid()->voxelSize,
                        &d_vertices,
                        &d_normals,
                        &triangleCount,
                        impl_->cudaStream  // ✅ FIX: Already void*, no cast needed
                    );
                    
                    if (success && triangleCount > 0) {
                        // TODO: Pass to MeshManager for D2D upload (next step!)
                        std::cout << "[GpuSculptTool] 🔄 Async mesh update: " 
                                  << triangleCount << " tris" << std::endl;
                        
                        // For now, free the buffers (temporary until MeshManager integration)
                        cudaFree(d_vertices);
                        cudaFree(d_normals);
                    }
                }
            }
        }
        */
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
    std::cout << "[DEBUG] applyBrushGPU called at (" << brushWorldPos.x << ", " 
              << brushWorldPos.y << ", " << brushWorldPos.z << ")" << std::endl;
    
    if (!impl_->gridIsOnGpu || impl_->currentGpuHandle == 0) {
        std::cout << "[DEBUG] FAILED: gridIsOnGpu=" << impl_->gridIsOnGpu 
                  << ", handle=" << impl_->currentGpuHandle << std::endl;
        return false;
    }
    
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
        std::cout << "[GpuSculptTool] ✅ GPU sculpt: " << elapsed << "ms" << std::endl;
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
