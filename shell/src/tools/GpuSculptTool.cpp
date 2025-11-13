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
#include <vector>
#include <openvdb/openvdb.h>
#include <chrono>
#include <functional>
#include <algorithm>



// После includes, перед namespace

struct AABB {
    glm::vec3 min, max;
    
    glm::vec3 center() const { return (min + max) * 0.5f; }
    
    bool intersect(const glm::vec3& rayOrigin, const glm::vec3& rayDir, float& tMin) const {
        glm::vec3 invDir = 1.0f / rayDir;
        glm::vec3 t0 = (min - rayOrigin) * invDir;
        glm::vec3 t1 = (max - rayOrigin) * invDir;
        glm::vec3 tmin = glm::min(t0, t1);
        glm::vec3 tmax = glm::max(t0, t1);
        float t_enter = std::max({tmin.x, tmin.y, tmin.z, 0.0f});
        float t_exit = std::min({tmax.x, tmax.y, tmax.z});
        if (t_enter > t_exit) return false;
        tMin = t_enter;
        return true;
    }
};

struct BVHNode {
    AABB bounds;
    int leftChild = -1;
    int rightChild = -1;
    int triangleStart = -1; // Для листьев
    int triangleCount = 0;
};



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
    
    // MODIFIED: Undo state now matches the CPU sculpt tool for correct unbounded support
    std::vector<float> gridDataBeforeStroke;
    openvdb::CoordBBox savedBeforeBBox;
    
    // Performance tracking
    int strokeCount = 0;
    float totalGpuTime = 0.0f;
    
    // НОВОЕ: Кеш меша для быстрого ray casting
    struct MeshCache {
        std::vector<glm::vec3> vertices;
        std::vector<unsigned int> indices;
        std::vector<int> triangleIndices; // Для BVH
        std::vector<BVHNode> bvh;
        bool valid = false;
    } meshCache;
    
    int raycastCount = 0;
    float totalRaycastTime = 0.0f;
    
    // Adaptive distance threshold
    float getMinBrushDistance() const {
        // Smaller brushes need finer control
        if (brushRadius < 0.5f) return brushRadius * 0.05f;
        if (brushRadius < 1.0f) return brushRadius * 0.10f;
        return brushRadius * 0.15f;
    }
    
    // НОВОЕ: Функция построения BVH
    void buildBVH() {
        if (meshCache.vertices.empty() || meshCache.indices.empty()) return;
        
        auto t_start = std::chrono::high_resolution_clock::now();
        
        meshCache.bvh.clear();
        size_t numTriangles = meshCache.indices.size() / 3;
        
        // Построить AABB для каждого треугольника
        std::vector<AABB> triBoxes(numTriangles);
        meshCache.triangleIndices.resize(numTriangles);
        
        for (size_t i = 0; i < numTriangles; ++i) {
            glm::vec3 v0 = meshCache.vertices[meshCache.indices[i * 3 + 0]];
            glm::vec3 v1 = meshCache.vertices[meshCache.indices[i * 3 + 1]];
            glm::vec3 v2 = meshCache.vertices[meshCache.indices[i * 3 + 2]];
            
            triBoxes[i].min = glm::min(glm::min(v0, v1), v2);
            triBoxes[i].max = glm::max(glm::max(v0, v1), v2);
            meshCache.triangleIndices[i] = i;
        }
        
        // Рекурсивное построение BVH
        std::function<int(int, int)> buildNode = [&](int start, int end) -> int {
            int nodeIdx = meshCache.bvh.size();
            meshCache.bvh.emplace_back();
            BVHNode& node = meshCache.bvh[nodeIdx];
            
            // Вычислить bounds для всех треугольников в диапазоне
            node.bounds = triBoxes[meshCache.triangleIndices[start]];
            for (int i = start + 1; i < end; ++i) {
                int triIdx = meshCache.triangleIndices[i];
                node.bounds.min = glm::min(node.bounds.min, triBoxes[triIdx].min);
                node.bounds.max = glm::max(node.bounds.max, triBoxes[triIdx].max);
            }
            
            int count = end - start;
            if (count <= 4) { // Leaf node
                node.triangleStart = start;
                node.triangleCount = count;
                return nodeIdx;
            }
            
            // Split по самой длинной оси
            glm::vec3 extent = node.bounds.max - node.bounds.min;
            int axis = 0;
            if (extent.y > extent.x) axis = 1;
            if (extent.z > extent[axis]) axis = 2;
            
            // Сортировка по центроиду
            std::sort(meshCache.triangleIndices.begin() + start, 
                     meshCache.triangleIndices.begin() + end,
                [&](int a, int b) {
                    return triBoxes[a].center()[axis] < triBoxes[b].center()[axis];
                });
            
            int mid = start + count / 2;
            node.leftChild = buildNode(start, mid);
            node.rightChild = buildNode(mid, end);
            
            return nodeIdx;
        };
        
        if (numTriangles > 0) {
            buildNode(0, numTriangles);
        }
        
        auto t_end = std::chrono::high_resolution_clock::now();
        float elapsed = std::chrono::duration<float, std::milli>(t_end - t_start).count();
        
        std::cout << "[GpuSculptTool] BVH built: " << meshCache.bvh.size() 
                  << " nodes, " << numTriangles << " tris in " 
                  << elapsed << "ms" << std::endl;
    }
    
    // НОВОЕ: Быстрый ray cast через BVH
    bool raycastBVH(const glm::vec3& rayOrigin, const glm::vec3& rayDir, 
                    glm::vec3& hitPoint, float& hitDist) {
        if (meshCache.bvh.empty()) return false;
        
        hitDist = std::numeric_limits<float>::max();
        bool hit = false;
        
        // Stack-based traversal (быстрее рекурсии)
        int stack[64];
        int stackPtr = 0;
        stack[stackPtr++] = 0;
        
        while (stackPtr > 0) {
            int nodeIdx = stack[--stackPtr];
            const BVHNode& node = meshCache.bvh[nodeIdx];
            
            float tMin;
            if (!node.bounds.intersect(rayOrigin, rayDir, tMin)) continue;
            if (tMin >= hitDist) continue;
            
            if (node.triangleCount > 0) { // Leaf - проверяем треугольники
                for (int i = 0; i < node.triangleCount; ++i) {
                    int triIdx = meshCache.triangleIndices[node.triangleStart + i];
                    glm::vec3 v0 = meshCache.vertices[meshCache.indices[triIdx * 3 + 0]];
                    glm::vec3 v1 = meshCache.vertices[meshCache.indices[triIdx * 3 + 1]];
                    glm::vec3 v2 = meshCache.vertices[meshCache.indices[triIdx * 3 + 2]];
                    
                    float t;
                    if (SnappingSystem::RayTriangleIntersect(rayOrigin, rayDir, v0, v1, v2, t)) {
                        if (t > 0 && t < hitDist) {
                            hitDist = t;
                            hit = true;
                        }
                    }
                }
            } else { // Internal node
                if (node.leftChild >= 0) stack[stackPtr++] = node.leftChild;
                if (node.rightChild >= 0) stack[stackPtr++] = node.rightChild;
            }
        }
        
        if (hit) {
            hitPoint = rayOrigin + rayDir * hitDist;
        }
        
        return hit;
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
    
    // НОВОЕ: Кешируем меш
    const auto& mesh = hitObject->getMeshBuffers();
    impl_->meshCache.vertices.clear();
    impl_->meshCache.vertices.reserve(mesh.vertices.size() / 3);
    
    for (size_t i = 0; i < mesh.vertices.size(); i += 3) {
        impl_->meshCache.vertices.emplace_back(
            mesh.vertices[i], mesh.vertices[i+1], mesh.vertices[i+2]
        );
    }
    
    impl_->meshCache.indices = mesh.indices;
    impl_->meshCache.valid = true;
    
    // НОВОЕ: Строим BVH (один раз!)
    impl_->buildBVH();
    
    // Точный ray cast с BVH
    glm::vec3 hitPosition;
    float hitDist;
    bool hit = impl_->raycastBVH(currentRayOrigin, currentRayDir, hitPosition, hitDist);
    
    if (!hit) {
        hitPosition = currentRayOrigin + currentRayDir * closestAABBDist;
    }
    
    // Get object and validate
    auto* obj = hitObject;
    if (!obj) return;



    auto* volGeo = dynamic_cast<Engine::VolumetricGeometry*>(obj->getGeometry());
    if (!volGeo) {
        std::cout << "[GpuSculptTool] ❌ Selected object is not volumetric" << std::endl;
        return;
    }



    // Start sculpting session
    impl_->activeObjectId = hitObject->get_id();
    impl_->lastHitPoint = hitPosition;
    impl_->lastBrushApplyPos = hitPosition;
    impl_->isSculpting = true;
    impl_->raycastCount = 0;
    impl_->totalRaycastTime = 0.0f;

    // MODIFIED: Capture state for undo, including the bounding box of active voxels
    impl_->savedBeforeBBox = volGeo->getGrid()->getActiveBounds();
    impl_->gridDataBeforeStroke = volGeo->getGrid()->toDenseArray();
    
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

            // NEW: Update logical dimensions to encompass all sculpted changes
            volGeo->getGrid()->updateDimensions();
            
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
    
    // MODIFIED: Create undo command with correct state data including bounding boxes
    if (!impl_->gridDataBeforeStroke.empty()) {
        // Capture the 'after' state
        openvdb::CoordBBox afterBBox = volGeo->getGrid()->getActiveBounds();
        std::vector<float> dataAfter = volGeo->getGrid()->toDenseArray();
        
        auto cmd = std::make_unique<Engine::SculptCommand>(
            impl_->context.scene,
            impl_->activeObjectId,
            std::move(impl_->gridDataBeforeStroke), // 'before' state was captured on mouse down
            std::move(dataAfter),
            impl_->savedBeforeBBox,                 // Pass the 'before' bounding box
            afterBBox                               // Pass the 'after' bounding box
        );
        
        impl_->context.scene->getCommandManager()->ExecuteCommand(std::move(cmd));
    }
    
    // НОВОЕ: Очистить кеш
    impl_->meshCache.valid = false;
    impl_->meshCache.bvh.clear();
    impl_->isSculpting = false;
    
    // Статистика
    if (impl_->raycastCount > 0) {
        float avgRaycast = impl_->totalRaycastTime / impl_->raycastCount;
        std::cout << "[GpuSculptTool] Avg raycast: " << avgRaycast 
                  << "ms (" << impl_->raycastCount << " total)" << std::endl;
    }
    
    // Clean up session state
    impl_->activeObjectId = 0;
    
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
    if (!impl_->isSculpting || !impl_->meshCache.valid) return;
    
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
    
    // НОВОЕ: Быстрый ray cast с BVH
    glm::vec3 hitPosition;
    float hitDist;
    
    auto t_start = std::chrono::high_resolution_clock::now();
    bool hit = impl_->raycastBVH(currentRayOrigin, currentRayDir, hitPosition, hitDist);
    auto t_end = std::chrono::high_resolution_clock::now();
    
    float elapsed = std::chrono::duration<float, std::milli>(t_end - t_start).count();
    impl_->totalRaycastTime += elapsed;
    impl_->raycastCount++;
    
    if (!hit) return; // Не попали в меш
    
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
