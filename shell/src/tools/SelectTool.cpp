#include "tools/SelectTool.h"
#include "engine/scene.h"
#include "engine/scene_object.h"
#include "cad_kernel/MeshBuffers.h"
#include "camera.h"
#include "snapping.h"
#include "engine/line.h"
#include "renderer.h"
#include <SDL2/SDL.h>
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp> // <-- FIX: Include header for distance2 and length2
#include <iostream>
#include <vector>
#include <set>
#include <list>
#include <map>
#include <algorithm>
#include <limits>
#include <chrono>

namespace { // Anonymous namespace for helpers local to this file

const float LINE_PICK_THRESHOLD_RADIUS = 0.25f;
const float CLICK_DRAG_THRESHOLD_SQ = 25.0f; // 5x5 pixels threshold

// REMOVED FindCoplanarAdjacentTriangles helper function. It's now obsolete.

// Helper to find the closest line segment to a ray
bool RayLineSegmentIntersection(
    const glm::vec3& rayOrigin, const glm::vec3& rayDir,
    const glm::vec3& p1, const glm::vec3& p2,
    float pickThresholdRadius,
    float& outDistanceAlongRay)
{
    const float LINE_RAY_EPSILON = 1e-6f;
    glm::vec3 segDir = p2 - p1;
    float segLenSq = glm::length2(segDir);

    if (segLenSq < LINE_RAY_EPSILON * LINE_RAY_EPSILON) { // It's a point
        outDistanceAlongRay = glm::dot(p1 - rayOrigin, rayDir);
        if (outDistanceAlongRay < 0) return false;
        glm::vec3 pointOnRay = rayOrigin + rayDir * outDistanceAlongRay;
        return glm::distance2(pointOnRay, p1) < pickThresholdRadius * pickThresholdRadius;
    }
    
    glm::vec3 segDirNormalized = glm::normalize(segDir);
    glm::vec3 w0 = rayOrigin - p1;
    
    float a = 1.0f;
    float b = glm::dot(rayDir, segDirNormalized);
    float c = 1.0f;
    float d = glm::dot(rayDir, w0);
    float e = glm::dot(segDirNormalized, w0);
    
    float denom = a * c - b * b;
    float t_seg_param;
    
    if (std::abs(denom) < LINE_RAY_EPSILON) { // Parallel lines
        return false;
    }
    
    t_seg_param = (a * e - b * d) / denom;
    
    // Clamp to segment
    float segActualLength = glm::sqrt(segLenSq);
    t_seg_param = glm::clamp(t_seg_param, 0.0f, segActualLength);
    
    glm::vec3 closestPointOnSegment = p1 + segDirNormalized * t_seg_param;
    float actual_t_ray = glm::dot(closestPointOnSegment - rayOrigin, rayDir);
    
    if (actual_t_ray < 0) return false;
    
    glm::vec3 closestPointOnRay = rayOrigin + actual_t_ray * rayDir;
    
    if (glm::distance2(closestPointOnRay, closestPointOnSegment) < pickThresholdRadius * pickThresholdRadius) {
        outDistanceAlongRay = actual_t_ray;
        return true;
    }
    return false;
}

// --- NEW: Helper for Axis-Aligned Bounding Box vs Line Segment intersection ---
bool AABBLineSegmentIntersect(const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& box_min, const glm::vec3& box_max) {
    glm::vec3 dir = p2 - p1;
    float tmin = 0.0f;
    float tmax = 1.0f;

    for (int i = 0; i < 3; ++i) {
        if (std::abs(dir[i]) < 1e-6) { // Parallel to the slab
            if (p1[i] < box_min[i] || p1[i] > box_max[i]) {
                return false; // Outside the slab, no intersection
            }
        } else {
            float ood = 1.0f / dir[i];
            float t1 = (box_min[i] - p1[i]) * ood;
            float t2 = (box_max[i] - p1[i]) * ood;
            if (t1 > t2) std::swap(t1, t2);

            tmin = std::max(tmin, t1);
            tmax = std::min(tmax, t2);

            if (tmin > tmax) {
                return false;
            }
        }
    }
    return true;
}

// --- NEW: Helper for AABB vs AABB intersection test ---
bool AABBvsAABB(const glm::vec3& minA, const glm::vec3& maxA, const glm::vec3& minB, const glm::vec3& maxB) {
    return (minA.x <= maxB.x && maxA.x >= minB.x) &&
           (minA.y <= maxB.y && maxA.y >= minB.y) &&
           (minA.z <= maxB.z && maxA.z >= minB.z);
}

} // end anonymous namespace

namespace Urbaxio::Tools {

SelectTool::SelectTool() {
    stopWorker_ = false;
    workerThread_ = std::thread(&SelectTool::worker_thread_main, this);
}

SelectTool::~SelectTool() {
    {
        std::unique_lock<std::mutex> lock(jobMutex_);
        stopWorker_ = true;
    }
    jobCondition_.notify_one();
    if (workerThread_.joinable()) {
        workerThread_.join();
    }
}

void SelectTool::Activate(const ToolContext& context) {
    ITool::Activate(context);
    lastClickTimestamp = 0;
    lastClickedObjId = 0;
    lastClickedTriangleIndex = 0;
    isMouseDown = false;
    isDragging = false;
    isVrTriggerDown = false;
    isVrDragging = false;
    vrTriggerDownTimestamp = 0;
    vrDragDistanceOffset = 0.0f;
    vrGhostPointAlpha_ = 0.0f;
}

void SelectTool::Deactivate() {
    isMouseDown = false;
    isDragging = false;
    isVrTriggerDown = false;
    isVrDragging = false;
    ITool::Deactivate();
}

void SelectTool::OnLeftMouseDown(int mouseX, int mouseY, bool shift, bool ctrl, const glm::vec3& rayOrigin, const glm::vec3& rayDirection) {
    // --- START OF MODIFICATION: Use explicit isVrMode flag ---
    // Check if we are in VR mode
    if (context.isVrMode) {
        isVrTriggerDown = true;
        isVrDragging = false;
        vrTriggerDownTimestamp = SDL_GetTicks();
        vrClickSnapResult = lastSnapResult; // Save snap result for a potential quick click

        // --- CORRECTED LOGIC: Set the start point immediately on click ---
        const float VISUAL_DISTANCE = 0.2f;
        float worldScale = context.worldTransform ? glm::length(glm::vec3((*context.worldTransform)[0])) : 1.0f;
        float worldDistance = (VISUAL_DISTANCE + vrDragDistanceOffset) * worldScale;
        vrDragStartPoint = rayOrigin + rayDirection * worldDistance;
        vrDragEndPoint = vrDragStartPoint; // Start and end are the same initially
    } else {
        // --- DESKTOP CLICK/DRAG START LOGIC ---
        isMouseDown = true;
        isDragging = false; // Reset drag state
        dragStartCoords = glm::vec2(mouseX, mouseY);
        currentDragCoords = dragStartCoords;
        
        if (!shift) {
            *context.selectedObjId = 0;
            context.selectedTriangleIndices->clear();
            context.selectedLineIDs->clear();
        }
    }
    // --- END OF MODIFICATION ---
}

void SelectTool::OnLeftMouseUp(int mouseX, int mouseY, bool shift, bool ctrl) {
    // NOTE: This function no longer handles VR drag finalization. It's done in FinalizeVrDragSelection.
    // It ONLY handles VR quick clicks and all desktop interactions.
    
    if (isVrDragging) {
        // This case is now handled by FinalizeVrDragSelection, so do nothing here.
        // We reset the state there.
        return;
    }

    if (isVrTriggerDown) {
        // This was a QUICK VR CLICK because the drag state was never initiated.
        isVrTriggerDown = false;

        // Perform single selection logic using the snap result we saved at MouseDown
        if (!shift) {
            *context.selectedObjId = 0;
            context.selectedTriangleIndices->clear();
            context.selectedLineIDs->clear();
        }

        if (vrClickSnapResult.snapped) {
            SnapType type = vrClickSnapResult.type;

            if (type == SnapType::ON_EDGE || type == SnapType::ENDPOINT || type == SnapType::MIDPOINT) {
                uint64_t lineId = vrClickSnapResult.snappedEntityId;
                if (lineId != 0) {
                    if (shift) {
                        if (context.selectedLineIDs->count(lineId)) context.selectedLineIDs->erase(lineId);
                        else context.selectedLineIDs->insert(lineId);
                    } else {
                        *context.selectedLineIDs = {lineId};
                    }
                }
            } else if (type == SnapType::ON_FACE) {
                uint64_t hitObjectId = vrClickSnapResult.snappedEntityId;
                Urbaxio::Engine::SceneObject* hitObject = context.scene->get_object_by_id(hitObjectId);
                if (hitObject) {
                    const auto& name = hitObject->get_name();
                    if (name == "LeftControllerVisual" || name == "RightControllerVisual") {
                        return; // Ignore controller visuals
                    }
                    int faceId = hitObject->getFaceIdForTriangle(vrClickSnapResult.snappedTriangleIndex);
                    const std::vector<size_t>* faceTriangles = (faceId != -1) ? hitObject->getFaceTriangles(faceId) : nullptr;
                    std::vector<size_t> newFace = faceTriangles ? *faceTriangles : std::vector<size_t>{vrClickSnapResult.snappedTriangleIndex};
                    if (shift) {
                        if (*context.selectedObjId == hitObjectId || *context.selectedObjId == 0) {
                            *context.selectedObjId = hitObjectId;
                            std::set<size_t> currentSelection(context.selectedTriangleIndices->begin(), context.selectedTriangleIndices->end());
                            bool isAlreadySelected = !newFace.empty() && currentSelection.count(newFace[0]);
                            if (isAlreadySelected) for(size_t idx : newFace) currentSelection.erase(idx);
                            else for(size_t idx : newFace) currentSelection.insert(idx);
                            context.selectedTriangleIndices->assign(currentSelection.begin(), currentSelection.end());
                        }
                    } else {
                        *context.selectedObjId = hitObjectId;
                        *context.selectedTriangleIndices = newFace;
                    }
                }
            }
        }
        return; // End of VR click logic
    }
    
    if (isDragging) {
        // --- It was a DRAG ---
        // --- NEW: Asynchronous Desktop Selection ---
        SelectionJob job;
        job.type = SelectionJob::JobType::RECT_2D;
        job.isWindowSelection = currentDragCoords.x >= dragStartCoords.x;
        job.rect_min = glm::min(dragStartCoords, currentDragCoords);
        job.rect_max = glm::max(dragStartCoords, currentDragCoords);
        job.shift = shift;
        // Capture context for the worker
        job.view = context.camera->GetViewMatrix();
        job.projection = context.camera->GetProjectionMatrix((float)*context.display_w / *context.display_h);
        job.screenWidth = *context.display_w;
        job.screenHeight = *context.display_h;
        // Gather data for the worker
        for (auto* obj : context.scene->get_all_objects()) {
            if (!obj || !obj->has_mesh()) continue;
            const auto& name = obj->get_name();
            if (name == "CenterMarker" || name == "UnitCapsuleMarker10m" || name == "UnitCapsuleMarker5m" || name == "LeftControllerVisual" || name == "RightControllerVisual") continue;
            
            // --- START OF MODIFICATION: Gather all data for worker (2D version) ---
            job.objectData[obj->get_id()] = {
                obj->get_mesh_buffers(),
                obj->getTriangleToFaceIDMap(),
                obj->getFacesMap(),
                obj->aabbMin,
                obj->aabbMax
            };
            // --- END OF MODIFICATION ---
        }
        job.allLines = context.scene->GetAllLines();
        // Send job to worker thread
        {
            std::unique_lock<std::mutex> lock(jobMutex_);
            jobQueue_.push(std::move(job));
        }
        jobCondition_.notify_one();
        
        // Clear current selection immediately for responsiveness
        if (!shift) {
            *context.selectedLineIDs = {};
            *context.selectedObjId = 0;
            *context.selectedTriangleIndices = {};
        }

    } else {
        // --- It was a CLICK ---
        if (!context.scene || !context.camera || !context.display_w || !context.display_h) return;

        glm::vec3 rayOrigin, rayDir;
        Camera::ScreenToWorldRay(mouseX, mouseY, *context.display_w, *context.display_h, context.camera->GetViewMatrix(), context.camera->GetProjectionMatrix((float)*context.display_w / (float)*context.display_h), rayOrigin, rayDir);

        uint64_t hitObjectId = 0; size_t hitTriangleBaseIndex = 0; float closestHitDistance = std::numeric_limits<float>::max();
        struct LineHit { uint64_t lineId; float distanceAlongRay; };
        std::vector<LineHit> lineHits;

        const auto& all_lines = context.scene->GetAllLines();
        for (const auto& [id, line] : all_lines) {
            float distAlongRay;
            if (RayLineSegmentIntersection(rayOrigin, rayDir, line.start, line.end, LINE_PICK_THRESHOLD_RADIUS, distAlongRay)) lineHits.push_back({id, distAlongRay});
        }

        if (!lineHits.empty()) {
            std::sort(lineHits.begin(), lineHits.end(), [](const LineHit& a, const LineHit& b) { return a.distanceAlongRay < b.distanceAlongRay; });
            if (shift) {
                if (context.selectedLineIDs->count(lineHits[0].lineId)) context.selectedLineIDs->erase(lineHits[0].lineId); else context.selectedLineIDs->insert(lineHits[0].lineId);
            } else {
                *context.selectedLineIDs = {lineHits[0].lineId};
            }
        } else {
             for (Urbaxio::Engine::SceneObject* obj_ptr : context.scene->get_all_objects()) {
                const auto& name = obj_ptr->get_name();
                if (obj_ptr && obj_ptr->has_mesh() && name != "CenterMarker" && name != "UnitCapsuleMarker10m" && name != "UnitCapsuleMarker5m" && name != "LeftControllerVisual" && name != "RightControllerVisual") {
                    const auto& mesh = obj_ptr->get_mesh_buffers();
                    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
                        glm::vec3 v0(mesh.vertices[mesh.indices[i] * 3], mesh.vertices[mesh.indices[i] * 3 + 1], mesh.vertices[mesh.indices[i] * 3 + 2]);
                        glm::vec3 v1(mesh.vertices[mesh.indices[i + 1] * 3], mesh.vertices[mesh.indices[i + 1] * 3 + 1], mesh.vertices[mesh.indices[i + 1] * 3 + 2]);
                        glm::vec3 v2(mesh.vertices[mesh.indices[i + 2] * 3], mesh.vertices[mesh.indices[i + 2] * 3 + 1], mesh.vertices[mesh.indices[i + 2] * 3 + 2]);
                        float t;
                        if (SnappingSystem::RayTriangleIntersect(rayOrigin, rayDir, v0, v1, v2, t) && t > 0 && t < closestHitDistance) {
                            closestHitDistance = t; hitObjectId = obj_ptr->get_id(); hitTriangleBaseIndex = i;
                        }
                    }
                }
            }
            if (hitObjectId != 0) {
                uint32_t currentTime = SDL_GetTicks();
                const uint32_t DOUBLE_CLICK_TIME = 300;
                if (currentTime - lastClickTimestamp < DOUBLE_CLICK_TIME && hitObjectId == lastClickedObjId && hitTriangleBaseIndex == lastClickedTriangleIndex) {
                    *context.selectedTriangleIndices = { hitTriangleBaseIndex }; *context.selectedObjId = hitObjectId; lastClickTimestamp = 0;
                } else {
                    Urbaxio::Engine::SceneObject* hitObject = context.scene->get_object_by_id(hitObjectId);
                    if (hitObject) {
                        // --- START OF MODIFICATION: Use pre-calculated faces ---
                        int faceId = hitObject->getFaceIdForTriangle(hitTriangleBaseIndex);
                        const std::vector<size_t>* faceTriangles = (faceId != -1) ? hitObject->getFaceTriangles(faceId) : nullptr;
                        
                        std::vector<size_t> newFace;
                        if (faceTriangles) {
                            newFace = *faceTriangles;
                        } else {
                            newFace = { hitTriangleBaseIndex }; // Fallback for objects without face data
                        }
                        // --- END OF MODIFICATION ---
                        if (shift) {
                            if (*context.selectedObjId == hitObjectId) {
                                std::set<size_t> currentSelection(context.selectedTriangleIndices->begin(), context.selectedTriangleIndices->end());
                                bool isAlreadySelected = !newFace.empty() && currentSelection.count(newFace[0]);
                                if (isAlreadySelected) for(size_t idx : newFace) currentSelection.erase(idx); else for(size_t idx : newFace) currentSelection.insert(idx);
                                context.selectedTriangleIndices->assign(currentSelection.begin(), currentSelection.end());
                            } else { *context.selectedObjId = hitObjectId; *context.selectedTriangleIndices = newFace; }
                        } else { *context.selectedObjId = hitObjectId; *context.selectedTriangleIndices = newFace; }
                    }
                    lastClickTimestamp = currentTime; lastClickedObjId = hitObjectId; lastClickedTriangleIndex = hitTriangleBaseIndex;
                }
            } else { lastClickTimestamp = 0; }
        }
    }

    isMouseDown = false;
    isDragging = false;
}

void SelectTool::FinalizeVrDragSelection(const glm::mat4& centerEyeViewMatrix, bool shift) {
    if (!isVrDragging) return;

    // --- VR 3D Box Selection Logic using the CORRECT view matrix ---
    glm::vec2 startScreen, endScreen;
    bool startVisible = SnappingSystem::WorldToScreen(vrDragStartPoint, centerEyeViewMatrix, context.camera->GetProjectionMatrix((float)*context.display_w / *context.display_h), *context.display_w, *context.display_h, startScreen);
    bool endVisible = SnappingSystem::WorldToScreen(vrDragEndPoint, centerEyeViewMatrix, context.camera->GetProjectionMatrix((float)*context.display_w / *context.display_h), *context.display_w, *context.display_h, endScreen);
    
    // The direction check is now reliable
    bool isWindowSelection = (startVisible && endVisible) ? (endScreen.x > startScreen.x) : true;
    
    SelectionJob job;
    // --- NEW: Set Job Type ---
    job.type = SelectionJob::JobType::BOX_3D;
    job.box_min = glm::min(vrDragStartPoint, vrDragEndPoint);
    job.box_max = glm::max(vrDragStartPoint, vrDragEndPoint);
    job.isWindowSelection = isWindowSelection;
    job.shift = shift;

    // --- START OF MODIFICATION: Gather all data for worker ---
    for (auto* obj : context.scene->get_all_objects()) {
        if (!obj || !obj->has_mesh()) continue;
        const auto& name = obj->get_name();
        if (name == "CenterMarker" || name == "UnitCapsuleMarker10m" || name == "UnitCapsuleMarker5m" || name == "LeftControllerVisual" || name == "RightControllerVisual") continue;
        
        job.objectData[obj->get_id()] = {
            obj->get_mesh_buffers(),
            obj->getTriangleToFaceIDMap(),
            obj->getFacesMap(),
            obj->aabbMin,
            obj->aabbMax
        };
    }
    // --- END OF MODIFICATION ---
    job.allLines = context.scene->GetAllLines();

    {
        std::unique_lock<std::mutex> lock(jobMutex_);
        jobQueue_.push(std::move(job));
    }
    jobCondition_.notify_one();
    
    // --- Apply Selection ---
    if (shift) {
        // This is now handled asynchronously
    } else {
        // This is now handled asynchronously, but we can clear the current selection immediately for responsiveness
        *context.selectedLineIDs = {};
        *context.selectedObjId = 0;
        *context.selectedTriangleIndices = {};
    }

    isVrTriggerDown = false;
    isVrDragging = false;
}

void SelectTool::OnMouseMove(int mouseX, int mouseY) {
    if (isMouseDown) {
        currentDragCoords = glm::vec2(mouseX, mouseY);
        if (!isDragging) {
            float dragDistanceSq = glm::distance2(dragStartCoords, currentDragCoords);
            if (dragDistanceSq > CLICK_DRAG_THRESHOLD_SQ) {
                isDragging = true;
            }
        }
    }
}

// --- NEW Method Implementations ---

bool SelectTool::IsDragging() const {
    return isDragging;
}

void SelectTool::GetDragRect(glm::vec2& outStart, glm::vec2& outCurrent) const {
    outStart = dragStartCoords;
    outCurrent = currentDragCoords;
}

void SelectTool::OnUpdate(const SnapResult& snap, const glm::vec3& rayOrigin, const glm::vec3& rayDirection) {
    ITool::OnUpdate(snap);

    {
        std::unique_lock<std::mutex> lock(resultMutex_, std::try_to_lock);
        if (lock.owns_lock() && !resultQueue_.empty()) {
            SelectionResult result = resultQueue_.front();
            resultQueue_.pop();

            if (result.shift) {
                for (uint64_t id : result.lineIDs) {
                    if (context.selectedLineIDs->count(id)) context.selectedLineIDs->erase(id);
                    else context.selectedLineIDs->insert(id);
                }
                if (result.objectId != 0 && (*context.selectedObjId == 0 || *context.selectedObjId == result.objectId)) {
                    *context.selectedObjId = result.objectId;
                    std::set<size_t> currentSelection(context.selectedTriangleIndices->begin(), context.selectedTriangleIndices->end());
                    for(size_t tri_idx : result.triangleIndices) {
                        if (currentSelection.count(tri_idx)) currentSelection.erase(tri_idx);
                        else currentSelection.insert(tri_idx);
                    }
                    context.selectedTriangleIndices->assign(currentSelection.begin(), currentSelection.end());
                }
            } else {
                *context.selectedLineIDs = result.lineIDs;
                *context.selectedObjId = result.objectId;
                *context.selectedTriangleIndices = result.triangleIndices;
            }
        }
    }

    float joyY = context.rightThumbstickY ? *context.rightThumbstickY : 0.0f;
    bool isJoystickActive = std::abs(joyY) > 0.1f;

    const float FADE_SPEED = 0.1f;
    float targetAlpha = (isJoystickActive || isVrDragging) ? 1.0f : 0.4f;
    if (isVrTriggerDown && !isVrDragging) {
        targetAlpha = 0.0f;
    }
    vrGhostPointAlpha_ += (targetAlpha - vrGhostPointAlpha_) * FADE_SPEED;

    if (isJoystickActive) {
        const float BASE_JOYSTICK_SPEED = 0.01f;
        const float ACCELERATION_FACTOR = 0.05f;
        const float MAX_DISTANCE_OFFSET = 100.0f;
        const float VISUAL_DISTANCE = 0.2f;
        const float MIN_DISTANCE_OFFSET = 0.01f - VISUAL_DISTANCE;

        float dynamicSpeed = BASE_JOYSTICK_SPEED + std::abs(vrDragDistanceOffset) * ACCELERATION_FACTOR;

        vrDragDistanceOffset += joyY * dynamicSpeed;

        vrDragDistanceOffset = std::clamp(vrDragDistanceOffset, MIN_DISTANCE_OFFSET, MAX_DISTANCE_OFFSET);
    }

    if (isVrTriggerDown) {
        // Check if we should transition to dragging state
        if (!isVrDragging) {
            const uint32_t VR_DRAG_DELAY_MS = 150;
            if (SDL_GetTicks() - vrTriggerDownTimestamp > VR_DRAG_DELAY_MS) {
                isVrDragging = true;
            }
        }
        
        // Always update the end point while the trigger is held down
        const float VISUAL_DISTANCE = 0.2f;
        float worldScale = context.worldTransform ? glm::length(glm::vec3((*context.worldTransform)[0])) : 1.0f;
        float worldDistance = (VISUAL_DISTANCE + vrDragDistanceOffset) * worldScale;

        // The start point is now fixed from MouseDown, only update the end point.
        vrDragEndPoint = rayOrigin + rayDirection * worldDistance;
    }
}

void SelectTool::RenderPreview(Renderer& renderer, const SnapResult& snap) {
    if (isVrDragging) {
        renderer.UpdatePreviewBox(vrDragStartPoint, vrDragEndPoint, true);
        renderer.UpdateDragStartPoint(vrDragStartPoint, true);
    } else {
        renderer.UpdatePreviewBox({}, {}, false);
        renderer.UpdateDragStartPoint({}, false);
    }
}

bool SelectTool::IsVrDragging() const {
    return isVrDragging;
}

bool SelectTool::IsVrTriggerDown() const {
    return isVrTriggerDown;
}

void SelectTool::GetVrDragBoxCorners(glm::vec3& outStart, glm::vec3& outEnd) const {
    outStart = vrDragStartPoint;
    outEnd = vrDragEndPoint;
}

float SelectTool::GetVrDragDistanceOffset() const {
    return vrDragDistanceOffset;
}

void SelectTool::GetVrGhostPoint(const glm::vec3& rayOrigin, const glm::vec3& rayDirection, glm::vec3& outPoint) const {
    const float VISUAL_DISTANCE = 0.2f;
    float worldScale = context.worldTransform ? glm::length(glm::vec3((*context.worldTransform)[0])) : 1.0f;
    float worldDistance = (VISUAL_DISTANCE + vrDragDistanceOffset) * worldScale;
    outPoint = rayOrigin + rayDirection * worldDistance;
}

float SelectTool::GetVrGhostPointAlpha() const {
    return vrGhostPointAlpha_;
}

void SelectTool::worker_thread_main() {
    while (true) {
        SelectionJob job;

        {
            std::unique_lock<std::mutex> lock(jobMutex_);
            jobCondition_.wait(lock, [this] { return !jobQueue_.empty() || stopWorker_; });

            if (stopWorker_) return;

            job = std::move(jobQueue_.front());
            jobQueue_.pop();
        }

        SelectionResult result;
        result.shift = job.shift;
        if (job.type == SelectionJob::JobType::BOX_3D) {
            // --- VR 3D BOX SELECTION LOGIC (Unchanged, but benefits from faster face grouping) ---
            auto is_point_in_box = [&](const glm::vec3& p) {
                return (p.x >= job.box_min.x && p.x <= job.box_max.x) &&
                       (p.y >= job.box_min.y && p.y <= job.box_max.y) &&
                       (p.z >= job.box_min.z && p.z <= job.box_max.z);
            };

            // Select Lines
            for (const auto& [id, line] : job.allLines) {
                bool start_in = is_point_in_box(line.start);
                bool end_in = is_point_in_box(line.end);
                if (job.isWindowSelection) {
                    if (start_in && end_in) result.lineIDs.insert(id);
                } else {
                    if (start_in || end_in || AABBLineSegmentIntersect(line.start, line.end, job.box_min, job.box_max)) {
                        result.lineIDs.insert(id);
                    }
                }
            }

            // Select Faces
            // --- START OF MODIFICATION: Optimized worker logic ---
            for (const auto& [objId, data] : job.objectData) {
                if (result.objectId != 0) break; // Only select from one object at a time
                // Optimization: Broad-phase check on the object's AABB
                if (!AABBvsAABB(job.box_min, job.box_max, data.aabbMin, data.aabbMax)) {
                    continue;
                }
                
                const auto& mesh = data.mesh;
                const size_t triCount = mesh.indices.size() / 3;
                std::vector<char> visited(triCount, 0);
                for (size_t i = 0; i < mesh.indices.size(); i += 3) {
                    if (visited[i / 3]) continue;
                    int faceId = data.triangleToFaceID[i / 3];
                    if (faceId == -1) { continue; } // Should not happen
                    
                    const auto& currentFace = data.faces.at(faceId);
                    for (size_t tri_idx : currentFace) visited[tri_idx / 3] = 1;
                    std::set<unsigned int> uniqueVertIndices;
                    for (size_t tri_idx : currentFace) {
                        uniqueVertIndices.insert(mesh.indices[tri_idx]);
                        uniqueVertIndices.insert(mesh.indices[tri_idx+1]);
                        uniqueVertIndices.insert(mesh.indices[tri_idx+2]);
                    }
                    bool all_in = true, any_in = false;
                    for (unsigned int v_idx : uniqueVertIndices) {
                        glm::vec3 v_pos(mesh.vertices[v_idx*3], mesh.vertices[v_idx*3+1], mesh.vertices[v_idx*3+2]);
                        if (is_point_in_box(v_pos)) any_in = true;
                        else all_in = false;
                    }
                    bool shouldSelectFace = false;
                    if (job.isWindowSelection) {
                        shouldSelectFace = all_in;
                    } else { // Crossing selection
                        if (any_in) {
                            shouldSelectFace = true;
                        } else {
                            // If no vertices are inside, check for edge intersections with the box
                            for(size_t tri_idx : currentFace) {
                                if (shouldSelectFace) break;
                                unsigned int i0 = mesh.indices[tri_idx], i1 = mesh.indices[tri_idx+1], i2 = mesh.indices[tri_idx+2];
                                glm::vec3 v0(mesh.vertices[i0*3], mesh.vertices[i0*3+1], mesh.vertices[i0*3+2]);
                                glm::vec3 v1(mesh.vertices[i1*3], mesh.vertices[i1*3+1], mesh.vertices[i1*3+2]);
                                glm::vec3 v2(mesh.vertices[i2*3], mesh.vertices[i2*3+1], mesh.vertices[i2*3+2]);
                                if (AABBLineSegmentIntersect(v0, v1, job.box_min, job.box_max) ||
                                    AABBLineSegmentIntersect(v1, v2, job.box_min, job.box_max) ||
                                    AABBLineSegmentIntersect(v2, v0, job.box_min, job.box_max)) {
                                    shouldSelectFace = true;
                                }
                            }
                        }
                    }
                    
                    if (shouldSelectFace) {
                        if (result.objectId == 0) result.objectId = objId;
                        result.triangleIndices.insert(result.triangleIndices.end(), currentFace.begin(), currentFace.end());
                    }
                }
            }
        } else if (job.type == SelectionJob::JobType::RECT_2D) {
            // --- NEW: 2D RECTANGLE SELECTION LOGIC ---
            auto is_point_in_rect = [&](const glm::vec2& p) {
                return p.x >= job.rect_min.x && p.x <= job.rect_max.x && p.y >= job.rect_min.y && p.y <= job.rect_max.y;
            };
            
            // Select Lines
            for (const auto& [id, line] : job.allLines) {
                glm::vec2 p1s, p2s;
                bool p1Visible = Urbaxio::SnappingSystem::WorldToScreen(line.start, job.view, job.projection, job.screenWidth, job.screenHeight, p1s);
                bool p2Visible = Urbaxio::SnappingSystem::WorldToScreen(line.end, job.view, job.projection, job.screenWidth, job.screenHeight, p2s);
                if (!p1Visible && !p2Visible) continue;
                if (job.isWindowSelection) {
                    if (is_point_in_rect(p1s) && is_point_in_rect(p2s)) result.lineIDs.insert(id);
                } else {
                    glm::vec2 lineMin = glm::min(p1s, p2s);
                    glm::vec2 lineMax = glm::max(p1s, p2s);
                    if (job.rect_max.x >= lineMin.x && job.rect_min.x <= lineMax.x && job.rect_max.y >= lineMin.y && job.rect_min.y <= lineMax.y) {
                         result.lineIDs.insert(id);
                    }
                }
            }

            // Select Faces
            for (const auto& [objId, data] : job.objectData) {
                if (result.objectId != 0) break;
                
                // Broad-phase (optional for 2D but can help) could go here
                
                const auto& mesh = data.mesh;
                const size_t triCount = mesh.indices.size() / 3;
                std::vector<char> visited(triCount, 0);
                for (size_t i = 0; i < mesh.indices.size(); i += 3) {
                    if (visited[i / 3]) continue;
                    
                    int faceId = data.triangleToFaceID[i / 3];
                    if (faceId == -1) { continue; }
                    
                    const auto& currentFace = data.faces.at(faceId);
                    for (size_t tri_idx : currentFace) visited[tri_idx / 3] = 1;
                    std::set<unsigned int> uniqueVertIndices;
                    for (size_t tri_idx : currentFace) {
                        uniqueVertIndices.insert(mesh.indices[tri_idx]);
                        uniqueVertIndices.insert(mesh.indices[tri_idx+1]);
                        uniqueVertIndices.insert(mesh.indices[tri_idx+2]);
                    }
                    bool all_in = true, any_in = false;
                    for (unsigned int v_idx : uniqueVertIndices) {
                        glm::vec3 v_world(mesh.vertices[v_idx*3], mesh.vertices[v_idx*3+1], mesh.vertices[v_idx*3+2]);
                        glm::vec2 v_screen;
                        if (Urbaxio::SnappingSystem::WorldToScreen(v_world, job.view, job.projection, job.screenWidth, job.screenHeight, v_screen)) {
                            if (is_point_in_rect(v_screen)) any_in = true; else all_in = false;
                        } else { all_in = false; }
                    }
                    
                    if (job.isWindowSelection ? all_in : any_in) {
                        if (result.objectId == 0) result.objectId = objId;
                        result.triangleIndices.insert(result.triangleIndices.end(), currentFace.begin(), currentFace.end());
                    }
                }
            }
        }

        {
            std::unique_lock<std::mutex> lock(resultMutex_);
            resultQueue_.push(std::move(result));
        }
    }
}

} // namespace Urbaxio::Tools 