#include "tools/SculptTool.h"
#include "engine/scene.h"
#include "engine/scene_object.h"
#include "engine/geometry/VolumetricGeometry.h"
#include "engine/geometry/VoxelGrid.h"
#include "renderer.h"
#include "snapping.h"
#include <imgui.h>
#include <iostream>
#include "camera.h"
#include "engine/commands/CommandManager.h"
#include "engine/commands/SculptCommand.h"
#include <memory>
#include <SDL2/SDL_mouse.h>
#include <limits>
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
    
    float closestHitDist = std::numeric_limits<float>::max();
    uint64_t hitObjectId = 0;
    glm::vec3 hitPosition;

    for (auto* obj : context.scene->get_all_objects()) {
        if (obj && dynamic_cast<Engine::VolumetricGeometry*>(obj->getGeometry())) {
            float t;
            if (obj->aabbValid && RayAABBIntersect(currentRayOrigin, currentRayDir, obj->aabbMin, obj->aabbMax, t)) {
                if (t < closestHitDist) {
                    closestHitDist = t;
                    hitObjectId = obj->get_id();
                    hitPosition = currentRayOrigin + currentRayDir * t;
                }
            }
        }
    }

    if (hitObjectId != 0) {
        isSculpting_ = true;
        sculptedObjectId_ = hitObjectId;
        
        Engine::SceneObject* obj = context.scene->get_object_by_id(sculptedObjectId_);
        auto* volGeom = obj ? dynamic_cast<Engine::VolumetricGeometry*>(obj->getGeometry()) : nullptr;
        if (volGeom && volGeom->getGrid()) {
            // --- CAPTURE BOTH 'BEFORE' AND 'WORKING' STATES ---
            gridDataBeforeStroke_ = volGeom->getGrid()->sdfData;
            workingGridData_ = volGeom->getGrid()->sdfData; // Start with a fresh copy
            
            std::cout << "[SculptTool] Mouse Down. Captured state for object " << sculptedObjectId_ << std::endl;
            
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
        sculptedObjectId_ = 0;
        gridDataBeforeStroke_.clear();
        workingGridData_.clear();
        return;
    }
    
    // The final state of our stroke is in the working copy
    auto command = std::make_unique<Engine::SculptCommand>(
        context.scene,
        sculptedObjectId_,
        std::move(gridDataBeforeStroke_),
        std::move(workingGridData_)
    );
    context.scene->getCommandManager()->ExecuteCommand(std::move(command));

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

    glm::vec3 brushPos = snap.worldPoint;
    bool hitSurface = false;
    float closestHitDist = std::numeric_limits<float>::max();
    
    if (sculptedObjectId_ != 0) { // If we are sculpting, only check the target object
        Engine::SceneObject* obj = context.scene->get_object_by_id(sculptedObjectId_);
        float t;
        if (obj && obj->aabbValid && RayAABBIntersect(currentRayOrigin, currentRayDir, obj->aabbMin, obj->aabbMax, t)) {
            closestHitDist = t;
            brushPos = currentRayOrigin + currentRayDir * t;
            hitSurface = true;
        }
    } else { // Otherwise, find a new target
        for (auto* obj : context.scene->get_all_objects()) {
            if (obj && dynamic_cast<Engine::VolumetricGeometry*>(obj->getGeometry())) {
                float t;
                if (obj->aabbValid && RayAABBIntersect(currentRayOrigin, currentRayDir, obj->aabbMin, obj->aabbMax, t)) {
                    if (t < closestHitDist) {
                        closestHitDist = t;
                        brushPos = currentRayOrigin + currentRayDir * t;
                        hitSurface = true;
                    }
                }
            }
        }
    }

    if (isSculpting_ && hitSurface) {
        // --- SOLUTION B: Distance Sampling ---
        const float MIN_BRUSH_DISTANCE_FACTOR = 0.25f; // Apply brush every 1/4 of its radius
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

    bool subtract = *context.ctrlDown; 
    
    // --- IMPORTANT: We now operate on the 'workingGridData_' copy ---
    Engine::SceneObject* obj = context.scene->get_object_by_id(sculptedObjectId_);
    auto* volGeom = obj ? dynamic_cast<Engine::VolumetricGeometry*>(obj->getGeometry()) : nullptr;
    if (!volGeom || !volGeom->getGrid()) return false;
    Engine::VoxelGrid* grid = volGeom->getGrid();
    
    glm::vec3 localPos = (brushWorldPos - grid->origin) / grid->voxelSize;
    float radiusInVoxels = brushRadius_ / grid->voxelSize;

    glm::ivec3 min_bound = glm::max(glm::ivec3(0), glm::ivec3(glm::floor(localPos - radiusInVoxels)));
    glm::ivec3 max_bound = glm::min(glm::ivec3(grid->dimensions) - 1, glm::ivec3(glm::ceil(localPos + radiusInVoxels)));
    
    bool gridModified = false;
    for (int z = min_bound.z; z <= max_bound.z; ++z) {
        for (int y = min_bound.y; y <= max_bound.y; ++y) {
            for (int x = min_bound.x; x <= max_bound.x; ++x) {
                // Read the original SDF from the 'before' state for a clean application
                size_t index = z * grid->dimensions.x * grid->dimensions.y + y * grid->dimensions.x + x;
                float original_sdf = gridDataBeforeStroke_[index];

                if (std::abs(original_sdf) > brushRadius_) {
                    continue;
                }

                glm::vec3 voxelLocalPos(x, y, z);
                float dist_to_brush_center = glm::distance(voxelLocalPos, localPos);

                if (dist_to_brush_center < radiusInVoxels) {
                    float falloff = 1.0f - (dist_to_brush_center / radiusInVoxels);
                    falloff = glm::smoothstep(0.0f, 1.0f, falloff);
                    
                    float displacement = brushStrength_ * falloff * grid->voxelSize * 10.0f; 

                    float& working_sdf = workingGridData_[index];
                    if (subtract) {
                        working_sdf = glm::max(working_sdf, original_sdf + displacement);
                    } else {
                        working_sdf = glm::min(working_sdf, original_sdf - displacement);
                    }
                    gridModified = true;
                }
            }
        }
    }
    
    // --- We need to temporarily update the real grid for live preview ---
    if (gridModified) {
        grid->sdfData = workingGridData_;
        obj->invalidateMeshCache();
        // IMPORTANT: DO NOT mark the scene as dirty here, it will cause massive slowdowns.
        // The final dirty flag is set by the command on mouse up.
    }

    return gridModified;
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

void SculptTool::RenderUI() {
    ImGui::Separator();
    ImGui::Text("Sculpt Settings");
    ImGui::SliderFloat("Radius", &brushRadius_, 0.1f, 10.0f);
    ImGui::SliderFloat("Strength", &brushStrength_, 0.01f, 1.0f);
    ImGui::TextDisabled("Hold Ctrl to Subtract");
    ImGui::Separator();
}

void SculptTool::RenderPreview(Renderer& renderer, const SnapResult& snap) {
    // The brush cursor is a regular SceneObject, it is rendered by the main renderer.
}

} // namespace Urbaxio::Tools
