#include "tools/SculptTool.h"
#include "engine/scene.h"
#include "engine/scene_object.h"
#include "engine/geometry/VolumetricGeometry.h"
#include "engine/geometry/VoxelGrid.h"
#include "renderer.h"
#include "snapping.h"
#include "LoadingManager.h"
#include <imgui.h>
#include <iostream>
#include "camera.h"
#include "engine/commands/CommandManager.h"
#include "engine/commands/SculptCommand.h"
#include <memory>
#include <SDL2/SDL_mouse.h>
#include <limits>
#include <openvdb/tools/Composite.h>
#include <openvdb/tools/LevelSetSphere.h>
#include <openvdb/math/Transform.h>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/geometric.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/norm.hpp>

namespace {
    // Ray-AABB intersection test (copied from snapping.cpp)
    bool RayAABBIntersect(
        const glm::vec3& rayOrigin, const glm::vec3& rayDir,
        const glm::vec3& boxMin, const glm::vec3& boxMax,
        float& t_intersect)
    {
        glm::vec3 invDir = 1.0f / rayDir;
        glm::vec3 t0s = (boxMin - rayOrigin) * invDir;
        glm::vec3 t1s = (boxMax - rayOrigin) * invDir;
        glm::vec3 tmin = glm::min(t0s, t1s);
        glm::vec3 tmax = glm::max(t0s, t1s);
        float t_enter = std::max({tmin.x, tmin.y, tmin.z});
        float t_exit = std::min({tmax.x, tmax.y, tmax.z});
        if (t_enter > t_exit || t_exit < 0) {
            return false;
        }
        t_intersect = t_enter;
        return true;
    }

    // Smooth minimum function for smooth union
    float smin(float a, float b, float k) {
        float h = glm::clamp(0.5f + 0.5f * (b - a) / k, 0.0f, 1.0f);
        return glm::mix(b, a, h) - k * h * (1.0f - h);
    }

    // Smooth maximum function for smooth subtraction
    float smax(float a, float b, float k) {
        return -smin(-a, -b, k);
    }
}

namespace Urbaxio::Tools {

void SculptTool::Activate(const ToolContext& context) {
    ITool::Activate(context);
    reset();

    for (auto* obj : this->context.scene->get_all_objects()) {
        if (obj && obj->get_name() == "BrushCursor") {
            brushCursorObjId_ = obj->get_id();
            break;
        }
    }
}

void SculptTool::Deactivate() {
    updateBrushCursor({}, false);
    reset();
    ITool::Deactivate();
}

void SculptTool::reset() {
    isSculpting_ = false;
    sculptedObjectId_ = 0;
    gridDataBeforeStroke_.clear();
    lastBrushApplyPos_ = glm::vec3(0.0f);
    workingGridData_.clear();
}

void SculptTool::OnLeftMouseDown(int mouseX, int mouseY, bool shift, bool ctrl, const glm::vec3& rayOrigin, const glm::vec3& rayDirection) {
    if (isSculpting_) return; // Already sculpting

    glm::vec3 currentRayOrigin, currentRayDir;
    if (context.isVrMode) {
        currentRayOrigin = rayOrigin;
        currentRayDir = rayDirection;
    } else {
        Camera::ScreenToWorldRay(mouseX, mouseY, *context.display_w, *context.display_h, context.camera->GetViewMatrix(), context.camera->GetProjectionMatrix((float)*context.display_w / (float)*context.display_h), currentRayOrigin, currentRayDir);
    }
    
    // Use ray-mesh intersection instead of ray-AABB to find the precise hit point
    float closestHitDist = std::numeric_limits<float>::max();
    uint64_t hitObjectId = 0;
    glm::vec3 hitPosition;
    bool hit = false;

    for (auto* obj : context.scene->get_all_objects()) {
        if (!obj || !dynamic_cast<Engine::VolumetricGeometry*>(obj->getGeometry()) || !obj->hasMesh()) {
            continue;
        }
        
        // Broad phase: check AABB first
        if (!obj->aabbValid) continue;
        float t_aabb;
        if (!RayAABBIntersect(currentRayOrigin, currentRayDir, obj->aabbMin, obj->aabbMax, t_aabb)) {
            continue;
        }

        // Narrow phase: check triangles
        const auto& mesh = obj->getMeshBuffers();
        for (size_t i = 0; i < mesh.indices.size(); i += 3) {
            unsigned int i0 = mesh.indices[i], i1 = mesh.indices[i+1], i2 = mesh.indices[i+2];
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

    if(hit) {
        hitPosition = currentRayOrigin + currentRayDir * closestHitDist;
    }

    if (hitObjectId != 0) {
        isSculpting_ = true;
        sculptedObjectId_ = hitObjectId;
        
        Engine::SceneObject* obj = context.scene->get_object_by_id(hitObjectId);
        auto* volGeom = obj ? dynamic_cast<Engine::VolumetricGeometry*>(obj->getGeometry()) : nullptr;
        if (volGeom && volGeom->getGrid()) {
            // NEW: Capture the active bounding box and the dense array relative to it
            savedBeforeBBox_ = volGeom->getGrid()->getActiveBounds();
            gridDataBeforeStroke_ = volGeom->getGrid()->toDenseArray();
            std::cout << "[SculptTool] Mouse Down. Captured state for object " << sculptedObjectId_ 
                      << " (" << volGeom->getGrid()->getActiveVoxelCount() << " active voxels)" << std::endl;
            
            applyBrush(hitPosition);
            lastBrushApplyPos_ = hitPosition;
        } else {
            isSculpting_ = false;
        }
    }
}

void SculptTool::OnLeftMouseUp(int mouseX, int mouseY, bool shift, bool ctrl) {
    if (!isSculpting_ || sculptedObjectId_ == 0) return;

    isSculpting_ = false;
    std::cout << "[SculptTool] Mouse Up. Finalizing sculpt command." << std::endl;

    Engine::SceneObject* obj = context.scene->get_object_by_id(sculptedObjectId_);
    auto* volGeom = obj ? dynamic_cast<Engine::VolumetricGeometry*>(obj->getGeometry()) : nullptr;
    if (!volGeom || !volGeom->getGrid()) {
        reset();
        return;
    }
    
    // NEW: Update logical dimensions to encompass all sculpted changes
    volGeom->getGrid()->updateDimensions();
    
    // The live grid is now in the "after" state. Capture it for the command.
    // NEW: Capture the new active bounding box and the corresponding dense array
    openvdb::CoordBBox afterBBox = volGeom->getGrid()->getActiveBounds();
    std::vector<float> dataAfter = volGeom->getGrid()->toDenseArray();
    
    auto command = std::make_unique<Engine::SculptCommand>(
        context.scene,
        sculptedObjectId_,
        std::move(gridDataBeforeStroke_), // 'before' state was captured on mouse down
        std::move(dataAfter),
        savedBeforeBBox_, // Pass the 'before' bounding box
        afterBBox         // Pass the 'after' bounding box
    );
    context.scene->getCommandManager()->ExecuteCommand(std::move(command));

    // Request async remesh. Crucially, we keep the old mesh for rendering until
    // the new one is ready. The AABB has already been updated in applyBrush,
    // so subsequent raycasts will work correctly.
    if (context.loadingManager) {
        context.loadingManager->RequestRemesh(context.scene, sculptedObjectId_);
        obj->markMeshAsClean(); // Prevents synchronous remesh
        std::cout << "[SculptTool] Async remesh requested." << std::endl;
    } else {
        obj->invalidateMeshCache(); // Fallback to synchronous remesh
        std::cout << "[SculptTool] WARNING: No LoadingManager, performing synchronous remesh." << std::endl;
    }
    sculptedObjectId_ = 0;
}

void SculptTool::OnUpdate(const SnapResult& snap, const glm::vec3& rayOrigin, const glm::vec3& rayDirection) {
    lastSnapResult = snap;
    
    glm::vec3 currentRayOrigin, currentRayDir;
    if (context.isVrMode) {
        currentRayOrigin = rayOrigin;
        currentRayDir = rayDirection;
    } else {
        int mouseX, mouseY;
        SDL_GetMouseState(&mouseX, &mouseY);
        Camera::ScreenToWorldRay(mouseX, mouseY, *context.display_w, *context.display_h, context.camera->GetViewMatrix(), context.camera->GetProjectionMatrix((float)*context.display_w / (float)*context.display_h), currentRayOrigin, currentRayDir);
    }

    glm::vec3 brushPos;
    bool hitSurface = false;
    float closestHitDist = std::numeric_limits<float>::max();
    
    // Use ray-mesh intersection instead of ray-AABB to find the precise hit point
    std::vector<Engine::SceneObject*> objectsToTest;
    if (sculptedObjectId_ != 0) { // If we are sculpting, only check the target object
        Engine::SceneObject* obj = context.scene->get_object_by_id(sculptedObjectId_);
        if (obj) objectsToTest.push_back(obj);
    } else { // Otherwise, find a new target among all volumetric objects
        for (auto* obj : context.scene->get_all_objects()) {
            if (obj && dynamic_cast<Engine::VolumetricGeometry*>(obj->getGeometry())) {
                objectsToTest.push_back(obj);
            }
        }
    }

    for (auto* obj : objectsToTest) {
        // Broad phase
        if (!obj->aabbValid || !obj->hasMesh()) continue;
        float t_aabb;
        if (!RayAABBIntersect(currentRayOrigin, currentRayDir, obj->aabbMin, obj->aabbMax, t_aabb)) {
            continue;
        }
        if (t_aabb > closestHitDist) continue; // AABB is further than an already found triangle hit

        // Narrow phase
        const auto& mesh = obj->getMeshBuffers();
        for (size_t i = 0; i < mesh.indices.size(); i += 3) {
            unsigned int i0 = mesh.indices[i], i1 = mesh.indices[i+1], i2 = mesh.indices[i+2];
            glm::vec3 v0(mesh.vertices[i0*3], mesh.vertices[i0*3+1], mesh.vertices[i0*3+2]);
            glm::vec3 v1(mesh.vertices[i1*3], mesh.vertices[i1*3+1], mesh.vertices[i1*3+2]);
            glm::vec3 v2(mesh.vertices[i2*3], mesh.vertices[i2*3+1], mesh.vertices[i2*3+2]);
            float t_tri;
            if (SnappingSystem::RayTriangleIntersect(currentRayOrigin, currentRayDir, v0, v1, v2, t_tri)) {
                if (t_tri > 0 && t_tri < closestHitDist) {
                    closestHitDist = t_tri;
                    hitSurface = true;
                }
            }
        }
    }
    
    if (hitSurface) {
        brushPos = currentRayOrigin + currentRayDir * closestHitDist;
    } else {
        // Fallback to snapping result if no mesh was hit (e.g. empty space)
        brushPos = snap.worldPoint;
    }

    if (isSculpting_ && hitSurface) {
        const float MIN_BRUSH_DISTANCE_FACTOR = 0.1f;
        float min_dist_sq = (brushRadius_ * MIN_BRUSH_DISTANCE_FACTOR) * (brushRadius_ * MIN_BRUSH_DISTANCE_FACTOR);

        if (glm::distance2(brushPos, lastBrushApplyPos_) > min_dist_sq) {
            applyBrush(brushPos);
            lastBrushApplyPos_ = brushPos;
        }
    }

    updateBrushCursor(brushPos, hitSurface);
}

bool SculptTool::applyBrush(const glm::vec3& brushWorldPos) {
    if (sculptedObjectId_ == 0) return false;

    Engine::SceneObject* obj = context.scene->get_object_by_id(sculptedObjectId_);
    auto* volGeom = obj ? dynamic_cast<Engine::VolumetricGeometry*>(obj->getGeometry()) : nullptr;
    if (!volGeom || !volGeom->getGrid()) return false;
    
    openvdb::FloatGrid::Ptr sceneGrid = volGeom->getGrid()->grid_;
    if (!sceneGrid) return false;
    // --- START OF RE-ARCHITECTURE: Correct CSG brush transformation ---
    
    // 1. Get the scene's voxel size to calculate brush radius in voxel units.
    float sceneVoxelSize = volGeom->getGrid()->voxelSize;
    float brushRadiusInVoxels = brushRadius_ / sceneVoxelSize;
    // 2. Convert the brush's world position into the scene grid's local index space.
    // This is the key step to align the brush with the scene.
    openvdb::Vec3d brushIndexPos = sceneGrid->transform().worldToIndex(
        openvdb::Vec3d(brushWorldPos.x, brushWorldPos.y, brushWorldPos.z)
    );
    // 3. Create a brush grid centered at the calculated INDEX position.
    // We use a voxel size of 1.0 because we are defining the sphere in index space.
    openvdb::FloatGrid::Ptr brushGrid =
        openvdb::tools::createLevelSetSphere<openvdb::FloatGrid>(
            brushRadiusInVoxels,                             // Radius in voxel units
            openvdb::Vec3f(brushIndexPos.x(), brushIndexPos.y(), brushIndexPos.z()), // Center in index units
            1.0f,                                            // Voxel size is 1.0 in index space
            brushRadiusInVoxels + 4.0f);                     // Use a wide narrow-band to prevent clipping
    
    // 4. CRITICAL FIX: Make the brush grid and scene grid share the exact same transform.
    // This tells the CSG operation that they exist in the same space, and the index-space
    // geometry we just created will align perfectly.
    brushGrid->setTransform(sceneGrid->transform().copy());
    brushGrid->setGridClass(openvdb::GRID_LEVEL_SET);
    // 5. Perform the CSG operation. Now that transforms match, this will work correctly.
    try {
        if (*context.ctrlDown) { // Remove material (A - B)
            openvdb::tools::csgDifference(*sceneGrid, *brushGrid);
        } else { // Add material (A u B)
            openvdb::tools::csgUnion(*sceneGrid, *brushGrid);
        }
        
        // Update the object's AABB immediately after modification
        openvdb::CoordBBox bbox = sceneGrid->evalActiveVoxelBoundingBox();
        if (!bbox.empty()) {
            openvdb::Vec3d worldMin = sceneGrid->indexToWorld(bbox.min());
            openvdb::Vec3d worldMax = sceneGrid->indexToWorld(bbox.max());
            obj->aabbMin = glm::vec3(worldMin.x(), worldMin.y(), worldMin.z());
            obj->aabbMax = glm::vec3(worldMax.x(), worldMax.y(), worldMax.z());
            obj->aabbValid = true;
        } else {
            obj->aabbValid = false; // Grid is empty
        }
    } catch (const openvdb::Exception& e) {
        std::cerr << "OpenVDB exception during CSG operation: " << e.what() << std::endl;
        return false;
    }
    // --- END OF RE-ARCHITECTURE ---
    return true;
}

void SculptTool::updateBrushCursor(const glm::vec3& position, bool visible) {
    if (brushCursorObjId_ == 0) return;
    Engine::SceneObject* cursorObj = context.scene->get_object_by_id(brushCursorObjId_);
    if (!cursorObj) return;

    if (visible) {
        glm::mat4 transform = glm::translate(glm::mat4(1.0f), position) * glm::scale(glm::mat4(1.0f), glm::vec3(brushRadius_));
        cursorObj->setTransform(transform);
    } else {
        cursorObj->setTransform(glm::scale(glm::mat4(1.0f), glm::vec3(0.0f)));
    }
}

// --- NEW: Setter Implementations ---
void SculptTool::SetBrushRadius(float radius) { 
    brushRadius_ = radius; 
}

void SculptTool::SetBrushStrength(float strength) { 
    brushStrength_ = strength; 
}
// -----------------------------------

void SculptTool::RenderUI() {
    ImGui::Separator();
    ImGui::Text("Sculpt Settings");
    ImGui::SliderFloat("Radius", &brushRadius_, 0.1f, 10.0f);
    ImGui::SliderFloat("Strength", &brushStrength_, 0.01f, 1.0f);
    ImGui::TextDisabled("Hold Ctrl to Subtract");
    ImGui::Separator();
    ImGui::TextDisabled("Tip: Use lower voxel resolution (64-128)");
    ImGui::TextDisabled("for smoother sculpting performance.");
}

void SculptTool::RenderPreview(Renderer& renderer, const SnapResult& snap) {
    // The brush cursor is a regular SceneObject, it is rendered by the main renderer.
}

} // namespace Urbaxio::Tools
