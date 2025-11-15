// VR Draw Tool - Using custom DynamicGpuHashGrid (unlimited + fast)
#include "tools/VrDrawTool.h"

#include "engine/Scene.h"
#include "engine/scene_object.h"
#include "engine/gpu/DynamicGpuHashGrid.h"
#include "engine/MeshManager.h"
#include "LoadingManager.h"
#include "imgui.h"

#include <iostream>
#include <cuda_runtime.h>
#include <chrono>
#include <optional>
#include <thread>
#include <atomic>

namespace Urbaxio::Shell {

// Static member definitions
std::unique_ptr<Engine::DynamicGpuHashGrid> VrDrawTool::s_sharedHashGrid = nullptr;
std::atomic<bool> VrDrawTool::s_resourcesReady{false};

struct VrDrawTool::Impl {
    Tools::ToolContext context;
    
    bool isDrawing = false;
    glm::vec3 lastDrawPos{0.0f};
    
    float brushRadius = 0.15f;
    float brushStrength = 1.0f;
    float minDrawDistanceFactor = 0.3f;
    
    // ✅ Ссылка на SHARED grid (не владеем!)
    Engine::DynamicGpuHashGrid* hashGrid = nullptr;
    uint64_t sceneObjectId = 0;
    
    void* cudaStream = nullptr;
    
    struct PendingMeshUpdate {
        void* cudaEvent = nullptr;
        float* d_vertices = nullptr;
        float* d_normals = nullptr;
        int* d_triangleCounter = nullptr;
    };
    std::optional<PendingMeshUpdate> pendingUpdate;
    
    int strokeCount = 0;
    float totalGpuTime = 0.0f;
    
    std::chrono::steady_clock::time_point lastMeshUpdate;
    float meshUpdateIntervalMs = 16.0f; // 60 FPS
    
    bool shouldUpdateMesh() {
        auto now = std::chrono::steady_clock::now();
        float elapsed = std::chrono::duration<float, std::milli>(now - lastMeshUpdate).count();
        return elapsed >= meshUpdateIntervalMs;
    }
    
    float getMinDrawDistance() const {
        return brushRadius * minDrawDistanceFactor;
    }
};

VrDrawTool::VrDrawTool() : impl_(std::make_unique<Impl>()) {
    cudaStreamCreate(reinterpret_cast<cudaStream_t*>(&impl_->cudaStream));
    std::cout << "[VrDrawTool] Initialized with DynamicGpuHashGrid (UNLIMITED + FAST)" << std::endl;
}

Tools::ToolType VrDrawTool::GetType() const {
    return Tools::ToolType::SculptDraw;
}

const char* VrDrawTool::GetName() const {
    return "VR Draw";
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

void VrDrawTool::InitializePersistentResources() {
    if (s_sharedHashGrid) {
        // Reset if already exists
        s_sharedHashGrid->Reset();
        s_resourcesReady = true;
        return;
    }
    
    std::cout << "[VrDrawTool] Allocating persistent GPU resources..." << std::endl;
    
    // 1. Create hash grid (reduced allocation for better performance)
    Engine::DynamicGpuHashGrid::Config config;
    config.voxelSize = 0.05f;
    config.maxBlocks = 200 * 1024;
    config.hashTableSize = 512 * 1024;
    s_sharedHashGrid = std::make_unique<Engine::DynamicGpuHashGrid>(config);
    
    // 2. Pre-warm kernels + allocate mesh buffers
    std::cout << "[VrDrawTool] Pre-warming GPU with dummy brush..." << std::endl;
    s_sharedHashGrid->ApplySphericalBrush(glm::vec3(0), 0.1f, 1.0f, true, nullptr);
    cudaDeviceSynchronize();
    std::cout << "[VrDrawTool] Pre-warm complete, active blocks: " 
              << s_sharedHashGrid->GetActiveBlockCount() << std::endl;
    
    float* dummy_v = nullptr;
    float* dummy_n = nullptr;
    int* dummy_c = nullptr;
    s_sharedHashGrid->ExtractMesh(0.0f, &dummy_v, &dummy_n, &dummy_c, nullptr);
    cudaDeviceSynchronize();
    
    s_resourcesReady = true;
    std::cout << "[VrDrawTool] ✅ Persistent resources ready (~450 MB)" << std::endl;
}

void VrDrawTool::CleanupPersistentResources() {
    s_sharedHashGrid.reset();
    s_resourcesReady = false;
}

void VrDrawTool::Activate(const Tools::ToolContext& context) {
    Tools::ITool::Activate(context);
    impl_->context = context;
    impl_->strokeCount = 0;
    impl_->totalGpuTime = 0.0f;

    // ✅ Just reference the shared grid (instant!)
    impl_->hashGrid = s_sharedHashGrid.get();
    
    std::cout << "[VrDrawTool] ✅ Activated (using pre-allocated resources)" << std::endl;
}

void VrDrawTool::Deactivate() {
    checkForAsyncUpdate(true);
    Tools::ITool::Deactivate();
    
    // ❌ DON'T delete shared grid!
    impl_->hashGrid = nullptr; // Just detach
    impl_->isDrawing = false;
    impl_->sceneObjectId = 0;
}

void VrDrawTool::OnTriggerPressed(bool, const glm::vec3& pos) {
    if (!impl_->context.scene) return;

    // ✅ Проверяем готовность ресурсов
    if (!s_resourcesReady.load()) {
        std::cout << "[VrDrawTool] ⏳ Resources still initializing, please wait..." << std::endl;
        return; // Игнорируем нажатие
    }

    if (!impl_->hashGrid) return;

    // Create a scene object to hold the mesh if needed
    if (impl_->sceneObjectId == 0) {
        auto* obj = impl_->context.scene->create_object("VR Drawing");
        impl_->sceneObjectId = obj->get_id();
    }
    
    impl_->isDrawing = true;
    impl_->lastDrawPos = pos;
    impl_->lastMeshUpdate = std::chrono::steady_clock::now();
    
    // Apply first stroke
    drawStroke(pos);
    
    std::cout << "[VrDrawTool] 🎨 Started drawing at (" << pos.x << ", " << pos.y << ", " << pos.z << ")" << std::endl;
}

void VrDrawTool::OnTriggerHeld(bool, const glm::vec3& pos) {
    if (!impl_->isDrawing || !impl_->hashGrid) return;
    
    float minDist = impl_->getMinDrawDistance();
    if (glm::distance(pos, impl_->lastDrawPos) >= minDist) {
        drawStroke(pos);
        impl_->lastDrawPos = pos;
        
        // ✅ Meshing handled in OnUpdate (frame-rate)
    }
}

void VrDrawTool::OnTriggerReleased(bool) {
    if (!impl_->isDrawing) return;
    
    std::cout << "[VrDrawTool] ✅ Drawing completed. Total strokes: " << impl_->strokeCount << std::endl;

    if (impl_->hashGrid) {
        impl_->hashGrid->PrintStats();
    }

    // ✅ Single mesh update at the end (non-blocking)
    std::cout << "[VrDrawTool] Generating final mesh..." << std::endl;
    updateMesh();

    impl_->isDrawing = false;
}

void VrDrawTool::updateMesh() {
    if (!impl_->hashGrid || impl_->sceneObjectId == 0) return;
    
    if (impl_->pendingUpdate) {
        std::cout << "[VrDrawTool] Mesh update already in progress, skipping." << std::endl;
        return;
    }
    
    auto* obj = impl_->context.scene->get_object_by_id(impl_->sceneObjectId);
    if (!obj) return;
    
    auto t_start = std::chrono::high_resolution_clock::now();
    
    std::cout << "[VrDrawTool] Requesting async mesh extraction..." << std::endl;
    
    float* d_vertices = nullptr;
    float* d_normals = nullptr;
    int* d_triangleCounter = nullptr;
    
    bool success = impl_->hashGrid->ExtractMesh(
        0.0f,
        &d_vertices,
        &d_normals,
        &d_triangleCounter,
        impl_->cudaStream
    );
    
    auto t_end = std::chrono::high_resolution_clock::now();
    float elapsed = std::chrono::duration<float, std::milli>(t_end - t_start).count();
    std::cout << "[VrDrawTool] ExtractMesh took: " << elapsed << "ms" << std::endl;
    
    if (!success) {
        std::cout << "[VrDrawTool] Mesh extraction failed to launch." << std::endl;
        return;
    }
    
    impl_->pendingUpdate.emplace();
    impl_->pendingUpdate->d_vertices = d_vertices;
    impl_->pendingUpdate->d_normals = d_normals;
    impl_->pendingUpdate->d_triangleCounter = d_triangleCounter;
    
    cudaEventCreate(reinterpret_cast<cudaEvent_t*>(&impl_->pendingUpdate->cudaEvent));
    cudaEventRecord(
        static_cast<cudaEvent_t>(impl_->pendingUpdate->cudaEvent),
        static_cast<cudaStream_t>(impl_->cudaStream)
    );
}

bool VrDrawTool::drawStroke(const glm::vec3& worldPos) {
    if (!impl_->hashGrid) return false;

    auto t_start = std::chrono::high_resolution_clock::now();

    bool success = impl_->hashGrid->ApplySphericalBrush(
        worldPos,
        impl_->brushRadius,
        impl_->brushStrength,
        true, // addMode
        impl_->cudaStream
    );

    auto t_end = std::chrono::high_resolution_clock::now();
    float elapsed = std::chrono::duration<float, std::milli>(t_end - t_start).count();

    if (success) {
        impl_->strokeCount++;
        impl_->totalGpuTime += elapsed;

        if (impl_->strokeCount % 100 == 0) {
            std::cout << "[VrDrawTool] Stroke " << impl_->strokeCount
                      << " - Avg time: " << (impl_->totalGpuTime / impl_->strokeCount) << "ms" << std::endl;
        }
    }

    return success;
}

void VrDrawTool::OnUpdate(const SnapResult&, const glm::vec3&, const glm::vec3&) {
    // ✅ Update cached count (async, no stall)
    if (impl_->hashGrid) {
        impl_->hashGrid->UpdateActiveCountAsync(impl_->cudaStream);
    }
    
    // ✅ Check for completed mesh
    checkForAsyncUpdate();
    
    // ✅ Start new mesh if needed (every 16ms)
    if (impl_->isDrawing && impl_->shouldUpdateMesh() && !impl_->pendingUpdate) {
        updateMesh();
        impl_->lastMeshUpdate = std::chrono::steady_clock::now();
    }
}

void VrDrawTool::checkForAsyncUpdate(bool forceSync) {
    if (!impl_->pendingUpdate) return;
    
    auto t_check = std::chrono::high_resolution_clock::now();
    
    cudaError_t eventStatus;
    if (forceSync) {
        std::cout << "[VrDrawTool] Force syncing event..." << std::endl;
        eventStatus = cudaEventSynchronize(static_cast<cudaEvent_t>(impl_->pendingUpdate->cudaEvent));
    } else {
        eventStatus = cudaEventQuery(static_cast<cudaEvent_t>(impl_->pendingUpdate->cudaEvent));
    }
    
    auto t_after = std::chrono::high_resolution_clock::now();
    float elapsed = std::chrono::duration<float, std::milli>(t_after - t_check).count();
    
    if (eventStatus == cudaSuccess) {
        std::cout << "[VrDrawTool] Event ready (took " << elapsed << "ms)" << std::endl;
        std::cout << "[VrDrawTool] Async mesh is ready, updating VBOs..." << std::endl;
        
        int triangleCount = 0;
        cudaMemcpy(&triangleCount, impl_->pendingUpdate->d_triangleCounter, sizeof(int), cudaMemcpyDeviceToHost);
        
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
                    
                    std::cout << "[VrDrawTool] ✅ Async mesh updated: " << triangleCount << " tris" << std::endl;
                }
            }
        }
        
        // ❌ Don't free vertices/normals (buffers are now persistent in hashGrid)
        // cudaFree(impl_->pendingUpdate->d_vertices);
        // cudaFree(impl_->pendingUpdate->d_normals);
        cudaFree(impl_->pendingUpdate->d_triangleCounter);  // ✅ Keep only this (small, allocated each time)
        cudaEventDestroy(static_cast<cudaEvent_t>(impl_->pendingUpdate->cudaEvent));
        impl_->pendingUpdate.reset();
        
    } else if (eventStatus != cudaErrorNotReady) {
        std::cerr << "[VrDrawTool] ❌ CUDA event error: " << cudaGetErrorString(eventStatus) << std::endl;
        // ❌ Don't free vertices/normals (buffers are persistent in hashGrid)
        cudaFree(impl_->pendingUpdate->d_triangleCounter);  // ✅ Only free counter
        cudaEventDestroy(static_cast<cudaEvent_t>(impl_->pendingUpdate->cudaEvent));
        impl_->pendingUpdate.reset();
    }
}

void VrDrawTool::RenderUI() {
    ImGui::Text("VR Draw Tool (Custom GPU Hash Grid)");
    ImGui::Separator();
    
    // ✅ Показываем loading indicator
    if (!s_resourcesReady.load()) {
        ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "⏳ Initializing GPU (first time)...");
        ImGui::TextDisabled("This happens once per session");
        return;
    }
        
        if (impl_->isDrawing) {
        ImGui::TextColored(ImVec4(0.0f, 1.0f, 1.0f, 1.0f), "🎨 Drawing...");
    } else {
        ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f), "Ready");
    }
    
    ImGui::Separator();
    ImGui::SliderFloat("Brush Radius", &impl_->brushRadius, 0.05f, 2.0f);
    ImGui::SliderFloat("Brush Strength", &impl_->brushStrength, 0.1f, 2.0f);
    ImGui::SliderFloat("Sample Distance", &impl_->minDrawDistanceFactor, 0.1f, 0.5f);
    
    ImGui::Separator();
    ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "⚡ Meshless Mode (real-time)");
    ImGui::TextDisabled("Mesh generated only at stroke end");
    
    ImGui::Separator();
    ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "✓ UNLIMITED range");
    ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "✓ Full GPU speed");
    ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "✓ Dynamic allocation");

    ImGui::Text("Total strokes: %d", impl_->strokeCount);

    if (impl_->strokeCount > 0) {
        ImGui::Text("Avg GPU time: %.3fms", impl_->totalGpuTime / impl_->strokeCount);
    }

    if (impl_->hashGrid) {
        ImGui::Separator();
        ImGui::Text("Active blocks: %u / %u",
                    impl_->hashGrid->GetActiveBlockCount(),
                    impl_->hashGrid->GetMaxBlocks());

        float fillRate = 100.0f * impl_->hashGrid->GetActiveBlockCount() / impl_->hashGrid->GetMaxBlocks();
        ImGui::ProgressBar(fillRate / 100.0f, ImVec2(-1, 0),
                          (std::to_string(static_cast<int>(fillRate)) + "%").c_str());
    }
}

} // namespace Urbaxio::Shell

