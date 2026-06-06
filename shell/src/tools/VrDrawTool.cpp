
#include "tools/VrDrawTool.h"
#include "engine/Scene.h"
#include "engine/scene_object.h"
#include "engine/gpu/DynamicGpuHashGrid.h"
#include "engine/MeshManager.h"
#include "LoadingManager.h"
#include "imgui.h"
#include "renderer.h"

#include <iostream>
#include <cuda_runtime.h>
#include <chrono>
#include <optional>
#include <thread>
#include <atomic>

namespace Urbaxio::Shell {

// ============================================================================
// STATIC MEMBERS
// ============================================================================

std::unique_ptr<Engine::DynamicGpuHashGrid> VrDrawTool::s_sharedHashGrid = nullptr;
std::atomic<bool> VrDrawTool::s_resourcesReady{false};

// ============================================================================
// IMPLEMENTATION STRUCT
// ============================================================================

struct VrDrawTool::Impl {
    Tools::ToolContext context;
    
    bool isDrawing = false;
    glm::vec3 lastDrawPos{0.0f};
    
    // Brush settings
    bool usePressure = true;
    float minRadius = 0.01f;
    float maxRadius = 0.15f;
    float brushStrength = 1.0f;
    bool isAdditive = true;
    bool isScaleDependent = false;
    
    float minDrawDistanceFactor = 0.3f;
    
    // Brush offset
    float brushOffsetDistance = 0.1f;
    glm::vec3 lastRayDirection{0.0f, 0.0f, -1.0f};
    glm::vec3 lastRayOrigin{0.0f};
    
    // Grid reference
    Engine::DynamicGpuHashGrid* hashGrid = nullptr;
    uint64_t sceneObjectId = 0;
    
    void* cudaStream = nullptr;
    
    // Async mesh update
    struct PendingMeshUpdate {
        void* cudaEvent = nullptr;
        float* d_vertices = nullptr;
        float* d_normals = nullptr;
        int* d_triangleCounter = nullptr;
        std::chrono::steady_clock::time_point startTime;
    };
    std::optional<PendingMeshUpdate> pendingUpdate;
    
    // Statistics
    int strokeCount = 0;
    float totalGpuTime = 0.0f;
    int meshUpdateCount = 0;
    float totalMeshTime = 0.0f;
    
    // ✅ NEW: Adaptive timing
    std::chrono::steady_clock::time_point lastMeshUpdate;
    float baseMeshUpdateIntervalMs = 33.0f;  // Base: 30 FPS mesh updates
    float currentMeshUpdateIntervalMs = 33.0f;
    
    // ✅ NEW: Performance tracking
    int lastTriangleCount = 0;
    float lastExtractTime = 0.0f;
    int consecutiveSlowFrames = 0;
    
    // ✅ NEW: Mesh update strategy
    enum class MeshUpdateStrategy {
        Aggressive,   // Update every frame (low triangle count)
        Normal,       // Update at base interval
        Conservative, // Update less frequently (high triangle count)
        Deferred      // Only update on trigger release
    };
    MeshUpdateStrategy meshStrategy = MeshUpdateStrategy::Normal;
    
    // ============================================================================
    // HELPER METHODS
    // ============================================================================
    
    bool shouldUpdateMesh() {
        auto now = std::chrono::steady_clock::now();
        float elapsed = std::chrono::duration<float, std::milli>(now - lastMeshUpdate).count();
        
        // Strategy-based decision
        switch (meshStrategy) {
            case MeshUpdateStrategy::Aggressive:
                return elapsed >= 16.0f;  // 60 FPS
                
            case MeshUpdateStrategy::Normal:
                return elapsed >= currentMeshUpdateIntervalMs;
                
            case MeshUpdateStrategy::Conservative:
                return elapsed >= currentMeshUpdateIntervalMs * 2.0f;
                
            case MeshUpdateStrategy::Deferred:
                return false;  // Only on trigger release
        }
        return elapsed >= currentMeshUpdateIntervalMs;
    }
    
    void updateMeshStrategy() {
        // Adjust strategy based on performance
        if (lastExtractTime > 20.0f) {  // > 20ms is slow
            consecutiveSlowFrames++;
            if (consecutiveSlowFrames > 3) {
                // Switch to more conservative strategy
                if (meshStrategy == MeshUpdateStrategy::Aggressive) {
                    meshStrategy = MeshUpdateStrategy::Normal;
                } else if (meshStrategy == MeshUpdateStrategy::Normal) {
                    meshStrategy = MeshUpdateStrategy::Conservative;
                } else if (meshStrategy == MeshUpdateStrategy::Conservative && lastTriangleCount > 1000000) {
                    meshStrategy = MeshUpdateStrategy::Deferred;
                    std::cout << "[VrDrawTool] Switching to deferred mesh updates (too many triangles)" << std::endl;
                }
                consecutiveSlowFrames = 0;
            }
        } else if (lastExtractTime < 5.0f) {  // < 5ms is fast
            consecutiveSlowFrames = 0;
            // Can be more aggressive
            if (meshStrategy == MeshUpdateStrategy::Conservative) {
                meshStrategy = MeshUpdateStrategy::Normal;
            } else if (meshStrategy == MeshUpdateStrategy::Normal && lastTriangleCount < 100000) {
                meshStrategy = MeshUpdateStrategy::Aggressive;
            }
        }
        
        // Adjust interval based on triangle count
        if (lastTriangleCount < 50000) {
            currentMeshUpdateIntervalMs = 16.0f;  // 60 FPS
        } else if (lastTriangleCount < 200000) {
            currentMeshUpdateIntervalMs = 33.0f;  // 30 FPS
        } else if (lastTriangleCount < 500000) {
            currentMeshUpdateIntervalMs = 50.0f;  // 20 FPS
        } else if (lastTriangleCount < 1000000) {
            currentMeshUpdateIntervalMs = 100.0f; // 10 FPS
        } else {
            currentMeshUpdateIntervalMs = 200.0f; // 5 FPS
        }
    }
    
    float getMinDrawDistance() const {
        return maxRadius * minDrawDistanceFactor; 
    }
    
    float getCurrentRadius(float worldScale) const {
        float baseRadius;
        
        if (!usePressure) {
            baseRadius = maxRadius;
        } else {
            float pressure = 1.0f;
            if (context.rightTriggerValue) {
                pressure = *context.rightTriggerValue;
            }
            baseRadius = glm::mix(minRadius, maxRadius, pressure);
        }
        
        if (isScaleDependent) {
            return baseRadius * worldScale;
        }
        return baseRadius;
    }
    
    const char* getStrategyName() const {
        switch (meshStrategy) {
            case MeshUpdateStrategy::Aggressive: return "Aggressive (60 FPS)";
            case MeshUpdateStrategy::Normal: return "Normal (30 FPS)";
            case MeshUpdateStrategy::Conservative: return "Conservative (15 FPS)";
            case MeshUpdateStrategy::Deferred: return "Deferred (on release)";
        }
        return "Unknown";
    }
};

// ============================================================================
// CONSTRUCTOR / DESTRUCTOR
// ============================================================================

VrDrawTool::VrDrawTool() : impl_(std::make_unique<Impl>()) {
    cudaStreamCreate(reinterpret_cast<cudaStream_t*>(&impl_->cudaStream));
    std::cout << "[VrDrawTool] Initialized (Optimized Version)" << std::endl;
}

VrDrawTool::~VrDrawTool() {
    checkForAsyncUpdate(true);
    
    if (impl_->cudaStream) {
        cudaStreamDestroy(static_cast<cudaStream_t>(impl_->cudaStream));
    }
    
    if (impl_->pendingUpdate && impl_->pendingUpdate->cudaEvent) {
        cudaEventDestroy(static_cast<cudaEvent_t>(impl_->pendingUpdate->cudaEvent));
    }
}

Tools::ToolType VrDrawTool::GetType() const {
    return Tools::ToolType::SculptDraw;
}

const char* VrDrawTool::GetName() const {
    return "VR Draw (Optimized)";
}

// ============================================================================
// STATIC INITIALIZATION
// ============================================================================

void VrDrawTool::InitializePersistentResources() {
    if (s_sharedHashGrid) {
        s_sharedHashGrid->Reset();
        s_resourcesReady = true;
        return;
    }
    
    std::cout << "[VrDrawTool] Allocating persistent GPU resources..." << std::endl;
    
    // ✅ Optimized config
    Engine::DynamicGpuHashGrid::Config config;
    config.voxelSize = 0.05f;
    config.maxBlocks = 200 * 1024;
    config.hashTableSize = 512 * 1024;
    config.maxDirtyBlocks = 8192;            // NEW
    config.enableIncrementalMesh = true;     // NEW
    
    s_sharedHashGrid = std::make_unique<Engine::DynamicGpuHashGrid>(config);
    
    // Minimal pre-warm with timeout protection
    std::cout << "[VrDrawTool] Pre-warming GPU kernels..." << std::endl;
    s_sharedHashGrid->ApplySphericalBrush(glm::vec3(0), 0.01f, 0.5f, true, nullptr);
    cudaError_t err = cudaDeviceSynchronize();
    if (err != cudaSuccess) {
        std::cerr << "[VrDrawTool] Pre-warm failed: " << cudaGetErrorString(err) << std::endl;
    }
    
    // Clear pre-warm data
    s_sharedHashGrid->Reset();
    
    s_resourcesReady = true;
    std::cout << "[VrDrawTool] ✅ Persistent resources ready (no ExtractMesh pre-warm)" << std::endl;
}

void VrDrawTool::CleanupPersistentResources() {
    s_sharedHashGrid.reset();
    s_resourcesReady = false;
}

// ============================================================================
// ACTIVATION
// ============================================================================

void VrDrawTool::Activate(const Tools::ToolContext& context) {
    Tools::ITool::Activate(context);
    impl_->context = context;
    impl_->strokeCount = 0;
    impl_->totalGpuTime = 0.0f;
    impl_->meshUpdateCount = 0;
    impl_->totalMeshTime = 0.0f;
    impl_->lastTriangleCount = 0;
    impl_->consecutiveSlowFrames = 0;
    impl_->meshStrategy = Impl::MeshUpdateStrategy::Normal;

    impl_->hashGrid = s_sharedHashGrid.get();
    
    std::cout << "[VrDrawTool] ✅ Activated (optimized mesh updates)" << std::endl;
}

void VrDrawTool::Deactivate() {
    checkForAsyncUpdate(true);
    Tools::ITool::Deactivate();

    if (impl_->context.renderer) {
        impl_->context.renderer->UpdateBrushPreview(glm::vec3(0), 0, glm::vec3(0), false);
    }
    
    impl_->hashGrid = nullptr;
    impl_->isDrawing = false;
    impl_->sceneObjectId = 0;
}

// ============================================================================
// TRIGGER EVENTS
// ============================================================================

void VrDrawTool::OnTriggerPressed(bool, const glm::vec3& pos) {
    if (!impl_->context.scene) return;
    if (!s_resourcesReady.load()) {
        std::cout << "[VrDrawTool] ⏳ Resources still initializing..." << std::endl;
        return;
    }
    if (!impl_->hashGrid) return;

    // Create scene object if needed
    if (impl_->sceneObjectId == 0) {
        auto* obj = impl_->context.scene->create_object("VR Drawing");
        impl_->sceneObjectId = obj->get_id();
    }
    
    impl_->isDrawing = true;
    impl_->lastMeshUpdate = std::chrono::steady_clock::now();
    
    // Reset strategy for new stroke
    impl_->meshStrategy = Impl::MeshUpdateStrategy::Normal;
    impl_->consecutiveSlowFrames = 0;
    
    // Calculate actual position with offset
    float worldScale = impl_->context.worldTransform 
        ? glm::length(glm::vec3((*impl_->context.worldTransform)[0])) 
        : 1.0f;
    glm::vec3 actualPos = pos + impl_->lastRayDirection * (impl_->brushOffsetDistance * worldScale);
    impl_->lastDrawPos = actualPos;

    // First stroke
    drawStroke(actualPos, worldScale);
    
    std::cout << "[VrDrawTool] 🎨 Started drawing" << std::endl;
}

void VrDrawTool::OnTriggerHeld(bool, const glm::vec3& pos) {
    if (!impl_->isDrawing || !impl_->hashGrid) return;
    
    float minDist = impl_->getMinDrawDistance();
    float worldScale = impl_->context.worldTransform 
        ? glm::length(glm::vec3((*impl_->context.worldTransform)[0])) 
        : 1.0f;
    
    if (impl_->isScaleDependent) {
        minDist *= worldScale;
    }

    glm::vec3 actualPos = pos + impl_->lastRayDirection * (impl_->brushOffsetDistance * worldScale);
    
    if (glm::distance(actualPos, impl_->lastDrawPos) >= minDist) {
        drawStroke(actualPos, worldScale);
        impl_->lastDrawPos = actualPos;
    }
}

void VrDrawTool::OnTriggerReleased(bool) {
    if (!impl_->isDrawing) return;
    
    std::cout << "[VrDrawTool] ✅ Drawing completed. Strokes: " << impl_->strokeCount 
              << ", Mesh updates: " << impl_->meshUpdateCount << std::endl;

    if (impl_->hashGrid) {
        impl_->hashGrid->PrintStats();
    }

    // Final mesh update (always do this)
    std::cout << "[VrDrawTool] Generating final mesh..." << std::endl;
    updateMesh();

    impl_->isDrawing = false;
}

// ============================================================================
// DRAWING
// ============================================================================

bool VrDrawTool::drawStroke(const glm::vec3& worldPos, float worldScale) {
    if (!impl_->hashGrid) return false;

    std::cout << "[DEBUG] drawStroke start" << std::endl;
    std::cout.flush();

    auto t_start = std::chrono::high_resolution_clock::now();

    float currentRadius = impl_->getCurrentRadius(worldScale);
    
    std::cout << "[DEBUG] calling ApplySphericalBrush r=" << currentRadius << std::endl;
    std::cout.flush();

    bool success = impl_->hashGrid->ApplySphericalBrush(
        worldPos,
        currentRadius,
        impl_->brushStrength,
        impl_->isAdditive,
        impl_->cudaStream
    );
    
    std::cout << "[DEBUG] ApplySphericalBrush done, syncing..." << std::endl;
    std::cout.flush();
    
    cudaDeviceSynchronize();  // Для отладки
    
    std::cout << "[DEBUG] sync done" << std::endl;
    std::cout.flush();

    auto t_end = std::chrono::high_resolution_clock::now();
    float elapsed = std::chrono::duration<float, std::milli>(t_end - t_start).count();

    if (success) {
        impl_->strokeCount++;
        impl_->totalGpuTime += elapsed;

        if (impl_->strokeCount % 100 == 0) {
            std::cout << "[VrDrawTool] Stroke " << impl_->strokeCount
                      << " - Avg brush time: " << (impl_->totalGpuTime / impl_->strokeCount) << "ms"
                      << " - Mesh strategy: " << impl_->getStrategyName() << std::endl;
        }
    }

    return success;
}

// ============================================================================
// MESH UPDATES
// ============================================================================

void VrDrawTool::updateMesh() {
    std::cout << "[DEBUG] updateMesh start" << std::endl;
    std::cout.flush();
    
    if (!impl_->hashGrid || impl_->sceneObjectId == 0) {
        std::cout << "[DEBUG] updateMesh early exit" << std::endl;
        return;
    }
    
    // Don't start new update if one is pending
    if (impl_->pendingUpdate) {
        std::cout << "[DEBUG] updateMesh: pending update exists" << std::endl;
        return;
    }
    
    auto* obj = impl_->context.scene->get_object_by_id(impl_->sceneObjectId);
    if (!obj) {
        std::cout << "[DEBUG] updateMesh: object not found" << std::endl;
        return;
    }
    
    std::cout << "[DEBUG] calling ExtractMesh..." << std::endl;
    std::cout.flush();
    
    auto t_start = std::chrono::high_resolution_clock::now();
    
    float* d_vertices = nullptr;
    float* d_normals = nullptr;
    int* d_triangleCounter = nullptr;
    
    bool success = impl_->hashGrid->ExtractMesh(
        0.0f,
        &d_vertices,
        &d_normals,
        &d_triangleCounter,
        nullptr  // stream
    );
    
    std::cout << "[DEBUG] ExtractMesh returned: " << success << std::endl;
    std::cout.flush();
    
    auto t_end = std::chrono::high_resolution_clock::now();
    impl_->lastExtractTime = std::chrono::duration<float, std::milli>(t_end - t_start).count();
    
    if (!success) {
        return;
    }
    
    // Setup async completion tracking
    impl_->pendingUpdate.emplace();
    impl_->pendingUpdate->d_vertices = d_vertices;
    impl_->pendingUpdate->d_normals = d_normals;
    impl_->pendingUpdate->d_triangleCounter = d_triangleCounter;
    impl_->pendingUpdate->startTime = t_start;
    
    cudaEventCreate(reinterpret_cast<cudaEvent_t*>(&impl_->pendingUpdate->cudaEvent));
    cudaEventRecord(
        static_cast<cudaEvent_t>(impl_->pendingUpdate->cudaEvent),
        static_cast<cudaStream_t>(impl_->cudaStream)
    );
    
    impl_->meshUpdateCount++;
}

void VrDrawTool::checkForAsyncUpdate(bool forceSync) {
    if (!impl_->pendingUpdate) return;
    
    cudaError_t eventStatus;
    if (forceSync) {
        eventStatus = cudaEventSynchronize(static_cast<cudaEvent_t>(impl_->pendingUpdate->cudaEvent));
    } else {
        eventStatus = cudaEventQuery(static_cast<cudaEvent_t>(impl_->pendingUpdate->cudaEvent));
    }
    
    if (eventStatus == cudaSuccess) {
        // Mesh is ready
        int triangleCount = 0;
        cudaMemcpy(&triangleCount, impl_->pendingUpdate->d_triangleCounter, sizeof(int), cudaMemcpyDeviceToHost);
        
        impl_->lastTriangleCount = triangleCount;
        
        if (triangleCount > 0) {
            auto* obj = impl_->context.scene->get_object_by_id(impl_->sceneObjectId);
            if (obj) {
                std::string meshId = std::to_string(impl_->sceneObjectId);
                auto* gpuMesh = impl_->context.meshManager->UpdateMeshFromGpuBuffers(
                    meshId,
                    impl_->pendingUpdate->d_vertices,
                    impl_->pendingUpdate->d_normals,
                    triangleCount * 3,
                    impl_->cudaStream
                );
                
                if (gpuMesh) {
                    obj->setGpuManaged(true);
                    obj->vao = gpuMesh->vao;
                    obj->index_count = gpuMesh->index_count;
                    
                    obj->meshGroups.clear();
                    Engine::MeshGroup defaultGroup;
                    defaultGroup.materialName = "Default";
                    defaultGroup.startIndex = 0;
                    defaultGroup.indexCount = gpuMesh->index_count;
                    obj->meshGroups.push_back(defaultGroup);
                    
                    impl_->context.scene->MarkStaticGeometryDirty();
                }
            }
        }
        
        // Update timing stats
        auto t_end = std::chrono::steady_clock::now();
        float totalTime = std::chrono::duration<float, std::milli>(
            t_end - impl_->pendingUpdate->startTime).count();
        impl_->totalMeshTime += totalTime;
        
        // Cleanup
        cudaFree(impl_->pendingUpdate->d_triangleCounter);
        cudaEventDestroy(static_cast<cudaEvent_t>(impl_->pendingUpdate->cudaEvent));
        impl_->pendingUpdate.reset();
        
        // Update strategy based on performance
        impl_->updateMeshStrategy();
        
    } else if (eventStatus != cudaErrorNotReady) {
        std::cerr << "[VrDrawTool] ❌ CUDA event error: " << cudaGetErrorString(eventStatus) << std::endl;
        cudaFree(impl_->pendingUpdate->d_triangleCounter);
        cudaEventDestroy(static_cast<cudaEvent_t>(impl_->pendingUpdate->cudaEvent));
        impl_->pendingUpdate.reset();
    }
}

// ============================================================================
// UPDATE LOOP
// ============================================================================

void VrDrawTool::OnUpdate(const SnapResult&, const glm::vec3& rayOrigin, const glm::vec3& rayDirection) {
    impl_->lastRayOrigin = rayOrigin;
    impl_->lastRayDirection = rayDirection;

    // Joystick control for brush offset
    float joyY = impl_->context.rightThumbstickY ? *impl_->context.rightThumbstickY : 0.0f;
    if (std::abs(joyY) > 0.1f) {
        const float BASE_SPEED = 0.01f;
        const float ACCEL = 0.05f;
        float dynamicSpeed = BASE_SPEED + std::abs(impl_->brushOffsetDistance) * ACCEL;
        impl_->brushOffsetDistance += joyY * dynamicSpeed;
        impl_->brushOffsetDistance = std::clamp(impl_->brushOffsetDistance, 0.01f, 100.0f);
    }
    
    // Update async count
    if (impl_->hashGrid) {
        impl_->hashGrid->UpdateActiveCountAsync(impl_->cudaStream);
    }
    
    // Check for completed mesh updates
    checkForAsyncUpdate();
    
    // Start new mesh update if needed (based on adaptive strategy)
    if (impl_->isDrawing && 
        impl_->strokeCount > 5 &&  // Подождать минимум 5 strokes
        impl_->shouldUpdateMesh() && 
        !impl_->pendingUpdate) 
    {
        updateMesh();
        impl_->lastMeshUpdate = std::chrono::steady_clock::now();
    }
}

// ============================================================================
// UI
// ============================================================================

void VrDrawTool::RenderUI() {
    ImGui::Text("VR Draw Tool (Optimized)");
    ImGui::Separator();
    
    if (!s_resourcesReady.load()) {
        ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "⏳ Initializing GPU...");
        return;
    }
    
    if (impl_->isDrawing) {
        ImGui::TextColored(ImVec4(0.0f, 1.0f, 1.0f, 1.0f), "🎨 Drawing...");
    } else {
        ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f), "Ready");
    }
    
    ImGui::Separator();
    ImGui::SliderFloat("Brush Radius", &impl_->maxRadius, 0.01f, 2.0f);
    ImGui::SliderFloat("Brush Strength", &impl_->brushStrength, 0.1f, 2.0f);
    ImGui::Checkbox("Pressure Sensitive", &impl_->usePressure);
    ImGui::Checkbox("Additive", &impl_->isAdditive);
    
    ImGui::Separator();
    ImGui::Text("Performance:");
    ImGui::Text("  Mesh Strategy: %s", impl_->getStrategyName());
    ImGui::Text("  Triangles: %d", impl_->lastTriangleCount);
    ImGui::Text("  Last Extract: %.2fms", impl_->lastExtractTime);
    ImGui::Text("  Update Interval: %.0fms", impl_->currentMeshUpdateIntervalMs);
    
    ImGui::Text("Statistics:");
    ImGui::Text("  Total strokes: %d", impl_->strokeCount);
    ImGui::Text("  Mesh updates: %d", impl_->meshUpdateCount);
    
    if (impl_->strokeCount > 0) {
        ImGui::Text("  Avg brush time: %.3fms", impl_->totalGpuTime / impl_->strokeCount);
    }
    if (impl_->meshUpdateCount > 0) {
        ImGui::Text("  Avg mesh time: %.2fms", impl_->totalMeshTime / impl_->meshUpdateCount);
    }

    if (impl_->hashGrid) {
        ImGui::Separator();
        auto stats = impl_->hashGrid->GetMeshStats();
        ImGui::Text("Active blocks: %u / %u", 
                    impl_->hashGrid->GetActiveBlockCount(),
                    impl_->hashGrid->GetMaxBlocks());
        
        float fillRate = 100.0f * impl_->hashGrid->GetActiveBlockCount() / impl_->hashGrid->GetMaxBlocks();
        ImGui::ProgressBar(fillRate / 100.0f, ImVec2(-1, 0),
                          (std::to_string(static_cast<int>(fillRate)) + "%").c_str());
    }
}

void VrDrawTool::RenderPreview(Renderer& renderer, const SnapResult& snap) {
    if (!impl_->hashGrid) return;

    float worldScale = impl_->context.worldTransform 
        ? glm::length(glm::vec3((*impl_->context.worldTransform)[0])) 
        : 1.0f;
    glm::vec3 previewPos = impl_->lastRayOrigin + impl_->lastRayDirection * (impl_->brushOffsetDistance * worldScale);
    
    renderer.UpdateBrushPreview(
        previewPos,
        impl_->getCurrentRadius(worldScale),
        glm::vec3(0.2f, 0.8f, 1.0f),
        true
    );
    renderer.UpdateDragStartPoint(previewPos, false);
}

// ============================================================================
// SETTERS
// ============================================================================

void VrDrawTool::SetPressureSensitivity(bool enabled) { impl_->usePressure = enabled; }
void VrDrawTool::SetBrushRadius(float radius) { impl_->maxRadius = radius; }
void VrDrawTool::SetMinBrushRadius(float radius) { impl_->minRadius = radius; }
void VrDrawTool::SetMaxBrushRadius(float radius) { impl_->maxRadius = radius; }
void VrDrawTool::SetBrushStrength(float strength) { impl_->brushStrength = strength; }
void VrDrawTool::SetAdditive(bool additive) { impl_->isAdditive = additive; }
void VrDrawTool::SetScaleDependent(bool enabled) { impl_->isScaleDependent = enabled; }

} // namespace Urbaxio::Shell