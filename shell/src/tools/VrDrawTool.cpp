// VR Draw Tool - Multi-chunk GPU system (unlimited + fast)

#include "tools/VrDrawTool.h"

#include "engine/Scene.h"
#include "engine/scene_object.h"
#include "engine/geometry/VolumetricGeometry.h"
#include "engine/geometry/VoxelGrid.h"
#include "engine/gpu/GpuVoxelManager.h"
#include "engine/gpu/GpuSculptKernels.h"
#include "engine/gpu/GpuMeshingKernels.h"
#include "engine/MeshManager.h"
#include "LoadingManager.h"
#include "imgui.h"

#include <openvdb/openvdb.h>
#include <cuda_runtime.h>
#include <iostream>
#include <string>

namespace Urbaxio::Shell {

struct ChunkInfo {
    uint64_t objectId = 0;
    uint64_t gpuHandle = 0;
    glm::ivec3 chunkCoord{0};
    openvdb::CoordBBox bounds;
};

struct VrDrawTool::Impl {
    Tools::ToolContext context;
    
    bool isDrawing = false;
    glm::vec3 lastDrawPos{0.0f};
    
    float brushRadius = 0.15f;
    float brushStrength = 1.0f;
    float minDrawDistanceFactor = 0.3f;
    
    std::vector<ChunkInfo> chunks;
    ChunkInfo* activeChunk = nullptr;
    
    Engine::GpuVoxelManager* gpuManager = nullptr;
    void* cudaStream = nullptr;
    
    int strokeCount = 0;
    float totalGpuTime = 0.0f;
    
    static constexpr int CHUNK_SIZE = 512;
    static constexpr float VOXEL_SIZE = 0.05f;
    
    float getMinDrawDistance() const {
        return brushRadius * minDrawDistanceFactor;
    }
    
    glm::ivec3 worldPosToChunkCoord(const glm::vec3& worldPos) const {
        float chunkWorldSize = CHUNK_SIZE * VOXEL_SIZE;
        return glm::ivec3(
            static_cast<int>(std::floor(worldPos.x / chunkWorldSize)),
            static_cast<int>(std::floor(worldPos.y / chunkWorldSize)),
            static_cast<int>(std::floor(worldPos.z / chunkWorldSize))
        );
    }
    
    ChunkInfo* findChunk(const glm::ivec3& coord) {
        for (auto& chunk : chunks) {
            if (chunk.chunkCoord == coord) return &chunk;
        }
        return nullptr;
    }
};

VrDrawTool::VrDrawTool() : impl_(std::make_unique<Impl>()) {
    cudaStreamCreate(reinterpret_cast<cudaStream_t*>(&impl_->cudaStream));
    std::cout << "[VrDrawTool] Multi-chunk GPU system initialized (UNLIMITED + FAST)" << std::endl;
}

VrDrawTool::~VrDrawTool() {
    if (impl_->cudaStream) {
        cudaStreamDestroy(static_cast<cudaStream_t>(impl_->cudaStream));
    }
    
    if (impl_->gpuManager) {
    delete impl_->gpuManager;
        impl_->gpuManager = nullptr;
    }
}

void VrDrawTool::Activate(const Tools::ToolContext& context) {
    Tools::ITool::Activate(context);
    impl_->context = context;
    
    if (!impl_->gpuManager) {
        impl_->gpuManager = new Engine::GpuVoxelManager();
    }
    
    impl_->strokeCount = 0;
    impl_->totalGpuTime = 0.0f;
    std::cout << "[VrDrawTool] Activated - ready for unlimited drawing!" << std::endl;
}

void VrDrawTool::Deactivate() {
    Tools::ITool::Deactivate();
    impl_->isDrawing = false;
    impl_->activeChunk = nullptr;
    impl_->chunks.clear();
}

Engine::SceneObject* VrDrawTool::createChunk(const glm::ivec3& chunkCoord) {
    std::cout << "[VrDrawTool] Creating chunk at " << chunkCoord.x << "," 
              << chunkCoord.y << "," << chunkCoord.z << std::endl;
    
    glm::vec3 chunkWorldOrigin(
        chunkCoord.x * impl_->CHUNK_SIZE * impl_->VOXEL_SIZE,
        chunkCoord.y * impl_->CHUNK_SIZE * impl_->VOXEL_SIZE,
        chunkCoord.z * impl_->CHUNK_SIZE * impl_->VOXEL_SIZE
    );
    
    glm::uvec3 dims(impl_->CHUNK_SIZE, impl_->CHUNK_SIZE, impl_->CHUNK_SIZE);
    auto grid = std::make_unique<Engine::VoxelGrid>(dims, chunkWorldOrigin, impl_->VOXEL_SIZE);
        
        auto floatGrid = openvdb::FloatGrid::create(1.0f);
    floatGrid->setTransform(
        openvdb::math::Transform::createLinearTransform(impl_->VOXEL_SIZE)
        );
        
    openvdb::Coord chunkOrigin(
        chunkCoord.x * impl_->CHUNK_SIZE,
        chunkCoord.y * impl_->CHUNK_SIZE,
        chunkCoord.z * impl_->CHUNK_SIZE
    );
        
        auto accessor = floatGrid->getAccessor();
    const int SHELL = 4;
    
    for (int face = 0; face < 6; ++face) {
        for (int u = 0; u < impl_->CHUNK_SIZE; ++u) {
            for (int v = 0; v < impl_->CHUNK_SIZE; ++v) {
                for (int s = 0; s < SHELL; ++s) {
                    openvdb::Coord ijk;
                    switch(face) {
                        case 0: ijk = chunkOrigin.offsetBy(s, u, v); break;
                        case 1: ijk = chunkOrigin.offsetBy(impl_->CHUNK_SIZE-1-s, u, v); break;
                        case 2: ijk = chunkOrigin.offsetBy(u, s, v); break;
                        case 3: ijk = chunkOrigin.offsetBy(u, impl_->CHUNK_SIZE-1-s, v); break;
                        case 4: ijk = chunkOrigin.offsetBy(u, v, s); break;
                        case 5: ijk = chunkOrigin.offsetBy(u, v, impl_->CHUNK_SIZE-1-s); break;
                    }
                    accessor.setValue(ijk, 1.0f);
                }
                }
            }
        }
        
        grid->grid_ = floatGrid;
        
    std::string chunkName = "Drawing Chunk " + std::to_string(chunkCoord.x) + "," + 
                            std::to_string(chunkCoord.y) + "," + std::to_string(chunkCoord.z);
    auto* obj = impl_->context.scene->create_object(chunkName);
        
        auto volGeo = std::make_unique<Engine::VolumetricGeometry>(std::move(grid));
    auto* vg = volGeo->getGrid();
        obj->setGeometry(std::move(volGeo));
    
    uint64_t handle = impl_->gpuManager->UploadGrid(vg);
    
    ChunkInfo info;
    info.objectId = obj->get_id();
    info.gpuHandle = handle;
    info.chunkCoord = chunkCoord;
    info.bounds = vg->getActiveBounds();
    impl_->chunks.push_back(info);
    
    std::cout << "[VrDrawTool] ✅ Chunk created with " 
              << floatGrid->activeVoxelCount() << " voxels" << std::endl;
    
    return obj;
    }
    
void VrDrawTool::OnTriggerPressed(bool isRightHand, const glm::vec3& pos) {
    glm::ivec3 chunkCoord = impl_->worldPosToChunkCoord(pos);
    
    impl_->activeChunk = impl_->findChunk(chunkCoord);
    
    if (!impl_->activeChunk) {
        createChunk(chunkCoord);
        impl_->activeChunk = &impl_->chunks.back();
    }
    
    impl_->isDrawing = true;
    impl_->lastDrawPos = pos;
    
    drawStroke(pos);
    
    std::cout << "[VrDrawTool] 🎨 Started drawing in chunk " << chunkCoord.x 
              << "," << chunkCoord.y << "," << chunkCoord.z << std::endl;
}

void VrDrawTool::OnTriggerHeld(bool isRightHand, const glm::vec3& pos) {
    if (!impl_->isDrawing) return;
    
    glm::ivec3 newChunkCoord = impl_->worldPosToChunkCoord(pos);
    
    if (impl_->activeChunk && newChunkCoord != impl_->activeChunk->chunkCoord) {
        std::cout << "[VrDrawTool] 🔄 Switching to chunk " << newChunkCoord.x 
                  << "," << newChunkCoord.y << "," << newChunkCoord.z << std::endl;
        
        impl_->activeChunk = impl_->findChunk(newChunkCoord);
        
        if (!impl_->activeChunk) {
            createChunk(newChunkCoord);
            impl_->activeChunk = &impl_->chunks.back();
        }
    }
    
    float minDist = impl_->getMinDrawDistance();
    if (glm::distance(pos, impl_->lastDrawPos) >= minDist) {
        drawStroke(pos);
        impl_->lastDrawPos = pos;
        
        static int counter = 0;
        if (++counter >= 5) {
            counter = 0;
            updateMeshRealtime();
        }
    }
}

void VrDrawTool::OnTriggerReleased(bool isRightHand) {
    if (!impl_->isDrawing) return;
    
    updateMeshRealtime();
    
        impl_->isDrawing = false;
    impl_->activeChunk = nullptr;
    
    std::cout << "[VrDrawTool] ✅ Drawing completed. Total chunks: " 
              << impl_->chunks.size() << std::endl;
}

bool VrDrawTool::drawStroke(const glm::vec3& worldPos) {
    if (!impl_->activeChunk) return false;
    
    auto* obj = impl_->context.scene->get_object_by_id(impl_->activeChunk->objectId);
    if (!obj) return false;
    
    auto* volGeo = dynamic_cast<Engine::VolumetricGeometry*>(obj->getGeometry());
    if (!volGeo) return false;
    
    auto* grid = volGeo->getGrid();
    
    openvdb::Vec3d indexPos = grid->grid_->transform().worldToIndex(
        openvdb::Vec3d(worldPos.x, worldPos.y, worldPos.z)
    );
    glm::vec3 brushPosIndex(indexPos.x(), indexPos.y(), indexPos.z());
    
    float brushRadiusVoxels = impl_->brushRadius / grid->voxelSize;
    
    void* devicePtr = impl_->gpuManager->GetDeviceGridPointer(impl_->activeChunk->gpuHandle);
    uint64_t leafCount = impl_->gpuManager->GetLeafCount(impl_->activeChunk->gpuHandle);
    
    auto bbox = grid->getActiveBounds();
    glm::ivec3 gridMin(bbox.min().x(), bbox.min().y(), bbox.min().z());
    glm::ivec3 gridMax(bbox.max().x(), bbox.max().y(), bbox.max().z());
    
    auto t_start = std::chrono::high_resolution_clock::now();
    
    bool success = Engine::GpuSculptKernels::ApplySphericalBrush(
        devicePtr,
        leafCount,
        brushPosIndex,
        brushRadiusVoxels,
        1.0f,
        Engine::GpuSculptKernels::SculptMode::ADD,
        impl_->brushStrength,
        gridMin, gridMax,
        nullptr, nullptr, nullptr,
        impl_->cudaStream
    );
    
    auto t_end = std::chrono::high_resolution_clock::now();
    float elapsed = std::chrono::duration<float, std::milli>(t_end - t_start).count();
    
    if (success) {
        impl_->strokeCount++;
        impl_->totalGpuTime += elapsed;
    }
    
    return success;
}

void VrDrawTool::updateMeshRealtime() {
    if (!impl_->activeChunk) return;
    
    auto* obj = impl_->context.scene->get_object_by_id(impl_->activeChunk->objectId);
    if (!obj) return;
    
    auto* volGeo = dynamic_cast<Engine::VolumetricGeometry*>(obj->getGeometry());
    if (!volGeo) return;
    
    uint64_t leafCount = impl_->gpuManager->GetLeafCount(impl_->activeChunk->gpuHandle);
    
    float* d_vertices = nullptr;
    float* d_normals = nullptr;
    int triangleCount = 0;
    
    bool meshSuccess = Engine::GpuMeshingKernels::MarchingCubesAsync(
        impl_->gpuManager->GetDeviceGridPointer(impl_->activeChunk->gpuHandle),
        leafCount,
        0.0f,
        volGeo->getGrid()->voxelSize,
        &d_vertices,
        &d_normals,
        &triangleCount,
        impl_->cudaStream
    );
    
    if (meshSuccess && triangleCount > 0) {
        cudaStreamSynchronize(static_cast<cudaStream_t>(impl_->cudaStream));
        
        std::string meshId = std::to_string(impl_->activeChunk->objectId);
        auto* gpuMesh = impl_->context.meshManager->UpdateMeshFromGpuBuffers(
            meshId, d_vertices, d_normals, triangleCount * 3, impl_->cudaStream
        );
        
        if (gpuMesh) {
            obj->setGpuManaged(true);
            obj->vao = gpuMesh->vao;
            obj->index_count = gpuMesh->index_count;
            
            obj->meshGroups.clear();
            Engine::MeshGroup group;
            group.materialName = "Default";
            group.startIndex = 0;
            group.indexCount = gpuMesh->index_count;
            obj->meshGroups.push_back(group);
            
            impl_->context.scene->MarkStaticGeometryDirty();
        }
        
        cudaFree(d_vertices);
        cudaFree(d_normals);
    }
}

void VrDrawTool::OnUpdate(const SnapResult&, const glm::vec3&, const glm::vec3&) {}

void VrDrawTool::RenderUI() {
    ImGui::Text("VR Draw Tool (Multi-Chunk GPU)");
    ImGui::Separator();
        
        if (impl_->isDrawing) {
        ImGui::TextColored(ImVec4(0.0f, 1.0f, 1.0f, 1.0f), "🎨 Drawing...");
        if (impl_->activeChunk) {
            ImGui::Text("Chunk: %d,%d,%d", impl_->activeChunk->chunkCoord.x,
                        impl_->activeChunk->chunkCoord.y, impl_->activeChunk->chunkCoord.z);
    }
    }
    
    ImGui::Separator();
    ImGui::SliderFloat("Radius", &impl_->brushRadius, 0.05f, 2.0f);
    ImGui::SliderFloat("Strength", &impl_->brushStrength, 0.1f, 2.0f);
    
    ImGui::Separator();
    ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "✓ UNLIMITED range");
    ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "✓ Full GPU speed");
    ImGui::Text("Total chunks: %zu", impl_->chunks.size());
    ImGui::Text("Strokes: %d", impl_->strokeCount);
    if (impl_->strokeCount > 0) {
        ImGui::Text("Avg GPU time: %.3fms", impl_->totalGpuTime / impl_->strokeCount);
    }
}

} // namespace Urbaxio::Shell

