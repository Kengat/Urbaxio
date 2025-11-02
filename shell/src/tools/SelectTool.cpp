#include "tools/SelectTool.h"
#include "engine/scene.h"
#include "engine/scene_object.h"
#include "cad_kernel/MeshBuffers.h"
#include "camera.h"
#include "snapping.h"
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

namespace { // Anonymous namespace for helpers local to this file

const float LINE_PICK_THRESHOLD_RADIUS = 0.25f;
const float CLICK_DRAG_THRESHOLD_SQ = 25.0f; // 5x5 pixels threshold

// Helper to find all coplanar and adjacent triangles, starting from a given one.
std::vector<size_t> FindCoplanarAdjacentTriangles(
    const Urbaxio::Engine::SceneObject& object,
    size_t startTriangleBaseIndex)
{
    // Do not attempt to group faces on special marker objects
    const auto& name = object.get_name();
    if (name == "CenterMarker" || name == "UnitCapsuleMarker10m" || name == "UnitCapsuleMarker5m") {
        return { startTriangleBaseIndex };
    }

    const float NORMAL_DOT_TOLERANCE = 0.999f; // Cosine of angle tolerance
    const float PLANE_DIST_TOLERANCE = 1e-4f;

    const auto& mesh = object.get_mesh_buffers();
    if (!object.has_mesh() || startTriangleBaseIndex + 2 >= mesh.indices.size()) {
        return { startTriangleBaseIndex };
    }

    // Build an edge-to-triangle map for adjacency lookups
    std::map<std::pair<unsigned int, unsigned int>, std::vector<size_t>> edgeToTriangles;
    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        unsigned int v_indices[3] = { mesh.indices[i], mesh.indices[i + 1], mesh.indices[i + 2] };
        for (int j = 0; j < 3; ++j) {
            unsigned int v1_idx = v_indices[j];
            unsigned int v2_idx = v_indices[(j + 1) % 3];
            if (v1_idx > v2_idx) std::swap(v1_idx, v2_idx);
            edgeToTriangles[{v1_idx, v2_idx}].push_back(i);
        }
    }

    std::vector<size_t> resultFaceTriangles;
    std::list<size_t> queue;
    std::set<size_t> visitedTriangles;

    // Define the reference plane from the starting triangle
    unsigned int i0 = mesh.indices[startTriangleBaseIndex];
    glm::vec3 v0(mesh.vertices[i0*3], mesh.vertices[i0*3+1], mesh.vertices[i0*3+2]);
    glm::vec3 referenceNormal(mesh.normals[i0*3], mesh.normals[i0*3+1], mesh.normals[i0*3+2]);
    float referencePlaneD = -glm::dot(referenceNormal, v0);

    // Start BFS
    queue.push_back(startTriangleBaseIndex);
    visitedTriangles.insert(startTriangleBaseIndex);

    while (!queue.empty()) {
        size_t currentTriangleIndex = queue.front();
        queue.pop_front();
        resultFaceTriangles.push_back(currentTriangleIndex);

        unsigned int current_v_indices[3] = { mesh.indices[currentTriangleIndex], mesh.indices[currentTriangleIndex + 1], mesh.indices[currentTriangleIndex + 2] };

        // Check neighbors through all 3 edges
        for (int j = 0; j < 3; ++j) {
            unsigned int v1_idx = current_v_indices[j];
            unsigned int v2_idx = current_v_indices[(j + 1) % 3];
            if (v1_idx > v2_idx) std::swap(v1_idx, v2_idx);

            const auto& potentialNeighbors = edgeToTriangles.at({v1_idx, v2_idx});
            for (size_t neighborIndex : potentialNeighbors) {
                if (neighborIndex == currentTriangleIndex) continue;

                if (visitedTriangles.find(neighborIndex) == visitedTriangles.end()) {
                    visitedTriangles.insert(neighborIndex);

                    unsigned int n_i0 = mesh.indices[neighborIndex];
                    glm::vec3 n_v0(mesh.vertices[n_i0*3], mesh.vertices[n_i0*3+1], mesh.vertices[n_i0*3+2]);
                    glm::vec3 neighborNormal(mesh.normals[n_i0*3], mesh.normals[n_i0*3+1], mesh.normals[n_i0*3+2]);
                    
                    // Check for coplanarity (normal and distance from plane)
                    if (glm::abs(glm::dot(referenceNormal, neighborNormal)) > NORMAL_DOT_TOLERANCE) {
                        float dist = glm::abs(glm::dot(referenceNormal, n_v0) + referencePlaneD);
                        if (dist < PLANE_DIST_TOLERANCE) {
                            queue.push_back(neighborIndex);
                        }
                    }
                }
            }
        }
    }
    return resultFaceTriangles;
}

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

} // end anonymous namespace

namespace Urbaxio::Tools {

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
}

void SelectTool::Deactivate() {
    isMouseDown = false;
    isDragging = false;
    isVrTriggerDown = false;
    isVrDragging = false;
    ITool::Deactivate();
}

void SelectTool::OnLeftMouseDown(int mouseX, int mouseY, bool shift, bool ctrl, const glm::vec3& rayOrigin, const glm::vec3& rayDirection) {
    // Check if a valid ray was passed (indicates VR path)
    if (glm::length2(rayDirection) > 1e-9) {
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
}

void SelectTool::OnLeftMouseUp(int mouseX, int mouseY, bool shift, bool ctrl) {
    if (isVrDragging) {
        // TODO: Future logic for 3D box selection will go here.
        isVrTriggerDown = false;
        isVrDragging = false;
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
                    std::vector<size_t> newFace = FindCoplanarAdjacentTriangles(*hitObject, vrClickSnapResult.snappedTriangleIndex);
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
        bool isCrossingSelection = currentDragCoords.x < dragStartCoords.x;
        
        glm::vec2 rectMin = glm::min(dragStartCoords, currentDragCoords);
        glm::vec2 rectMax = glm::max(dragStartCoords, currentDragCoords);

        auto isPointInRect = [&](const glm::vec2& p) {
            return p.x >= rectMin.x && p.x <= rectMax.x && p.y >= rectMin.y && p.y <= rectMax.y;
        };

        // --- Select Lines ---
        std::set<uint64_t> newlySelectedLines;
        for (const auto& [id, line] : context.scene->GetAllLines()) {
            glm::vec2 p1s, p2s;
            bool p1Visible = SnappingSystem::WorldToScreen(line.start, context.camera->GetViewMatrix(), context.camera->GetProjectionMatrix((float)*context.display_w / *context.display_h), *context.display_w, *context.display_h, p1s);
            bool p2Visible = SnappingSystem::WorldToScreen(line.end, context.camera->GetViewMatrix(), context.camera->GetProjectionMatrix((float)*context.display_w / *context.display_h), *context.display_w, *context.display_h, p2s);

            if (!p1Visible && !p2Visible) continue; 

            if (isCrossingSelection) {
                glm::vec2 lineMin = glm::min(p1s, p2s);
                glm::vec2 lineMax = glm::max(p1s, p2s);
                if (rectMax.x >= lineMin.x && rectMin.x <= lineMax.x && rectMax.y >= lineMin.y && rectMin.y <= lineMax.y) {
                    newlySelectedLines.insert(id);
                }
            } else { // Window selection
                if (isPointInRect(p1s) && isPointInRect(p2s)) {
                    newlySelectedLines.insert(id);
                }
            }
        }

        // --- Select Faces ---
        std::vector<size_t> newlySelectedTriangles;
        uint64_t newlySelectedObjId = 0;

        for (auto* obj : context.scene->get_all_objects()) {
            if (!obj || !obj->has_mesh()) continue;
            const auto& name = obj->get_name();
            if (name == "CenterMarker" || name == "UnitCapsuleMarker10m" || name == "UnitCapsuleMarker5m") continue;
            
            std::set<size_t> processedTrianglesInObject;
            const auto& mesh = obj->get_mesh_buffers();
            for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
                if(processedTrianglesInObject.count(i)) continue;

                std::vector<size_t> currentFace = FindCoplanarAdjacentTriangles(*obj, i);
                for(size_t tri_idx : currentFace) processedTrianglesInObject.insert(tri_idx);

                bool shouldSelectFace = false;
                if(isCrossingSelection) {
                    bool anyVertexInRect = false;
                    for(size_t tri_idx : currentFace) {
                        unsigned int i0 = mesh.indices[tri_idx], i1 = mesh.indices[tri_idx+1], i2 = mesh.indices[tri_idx+2];
                        glm::vec3 v_world[] = {
                            {mesh.vertices[i0*3], mesh.vertices[i0*3+1], mesh.vertices[i0*3+2]},
                            {mesh.vertices[i1*3], mesh.vertices[i1*3+1], mesh.vertices[i1*3+2]},
                            {mesh.vertices[i2*3], mesh.vertices[i2*3+1], mesh.vertices[i2*3+2]}
                        };
                        glm::vec2 v_screen[3];
                        bool v_vis[] = {
                            SnappingSystem::WorldToScreen(v_world[0], context.camera->GetViewMatrix(), context.camera->GetProjectionMatrix((float)*context.display_w / *context.display_h), *context.display_w, *context.display_h, v_screen[0]),
                            SnappingSystem::WorldToScreen(v_world[1], context.camera->GetViewMatrix(), context.camera->GetProjectionMatrix((float)*context.display_w / *context.display_h), *context.display_w, *context.display_h, v_screen[1]),
                            SnappingSystem::WorldToScreen(v_world[2], context.camera->GetViewMatrix(), context.camera->GetProjectionMatrix((float)*context.display_w / *context.display_h), *context.display_w, *context.display_h, v_screen[2])
                        };
                        if((v_vis[0] && isPointInRect(v_screen[0])) || (v_vis[1] && isPointInRect(v_screen[1])) || (v_vis[2] && isPointInRect(v_screen[2]))) {
                            anyVertexInRect = true; break;
                        }
                    }
                    if(anyVertexInRect) shouldSelectFace = true;
                } else { // Window selection
                    bool allVerticesInRect = true;
                    for(size_t tri_idx : currentFace) {
                        unsigned int i0 = mesh.indices[tri_idx], i1 = mesh.indices[tri_idx+1], i2 = mesh.indices[tri_idx+2];
                        glm::vec3 v_world[] = {
                            {mesh.vertices[i0*3], mesh.vertices[i0*3+1], mesh.vertices[i0*3+2]},
                            {mesh.vertices[i1*3], mesh.vertices[i1*3+1], mesh.vertices[i1*3+2]},
                            {mesh.vertices[i2*3], mesh.vertices[i2*3+1], mesh.vertices[i2*3+2]}
                        };
                        glm::vec2 v_screen[3];
                        bool v_vis[] = {
                             SnappingSystem::WorldToScreen(v_world[0], context.camera->GetViewMatrix(), context.camera->GetProjectionMatrix((float)*context.display_w / *context.display_h), *context.display_w, *context.display_h, v_screen[0]),
                             SnappingSystem::WorldToScreen(v_world[1], context.camera->GetViewMatrix(), context.camera->GetProjectionMatrix((float)*context.display_w / *context.display_h), *context.display_w, *context.display_h, v_screen[1]),
                             SnappingSystem::WorldToScreen(v_world[2], context.camera->GetViewMatrix(), context.camera->GetProjectionMatrix((float)*context.display_w / *context.display_h), *context.display_w, *context.display_h, v_screen[2])
                        };
                        if(!v_vis[0] || !isPointInRect(v_screen[0]) || !v_vis[1] || !isPointInRect(v_screen[1]) || !v_vis[2] || !isPointInRect(v_screen[2])) { 
                            allVerticesInRect = false; break;
                        }
                    }
                    if(allVerticesInRect) shouldSelectFace = true;
                }

                if(shouldSelectFace) {
                    if (newlySelectedObjId == 0) newlySelectedObjId = obj->get_id();
                    if (obj->get_id() == newlySelectedObjId) {
                        newlySelectedTriangles.insert(newlySelectedTriangles.end(), currentFace.begin(), currentFace.end());
                    }
                }
            }
            if (newlySelectedObjId != 0) break;
        }

        // --- Apply Selection ---
        if (shift) {
            for (uint64_t id : newlySelectedLines) { if (context.selectedLineIDs->count(id)) context.selectedLineIDs->erase(id); else context.selectedLineIDs->insert(id); }
            if (newlySelectedObjId != 0 && (*context.selectedObjId == 0 || *context.selectedObjId == newlySelectedObjId)) {
                *context.selectedObjId = newlySelectedObjId;
                std::set<size_t> currentSelection(context.selectedTriangleIndices->begin(), context.selectedTriangleIndices->end());
                for(size_t tri_idx : newlySelectedTriangles) { if (currentSelection.count(tri_idx)) currentSelection.erase(tri_idx); else currentSelection.insert(tri_idx); }
                context.selectedTriangleIndices->assign(currentSelection.begin(), currentSelection.end());
            }
        } else {
            *context.selectedLineIDs = newlySelectedLines; *context.selectedObjId = newlySelectedObjId; *context.selectedTriangleIndices = newlySelectedTriangles;
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
                if (obj_ptr && obj_ptr->has_mesh() && name != "CenterMarker" && name != "UnitCapsuleMarker10m" && name != "UnitCapsuleMarker5m") {
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
                        std::vector<size_t> newFace = FindCoplanarAdjacentTriangles(*hitObject, hitTriangleBaseIndex);
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
    ITool::OnUpdate(snap); // Store lastSnapResult for quick clicks

    // --- MODIFIED: Joystick logic is now always active for this tool ---
    if (context.rightThumbstickY) {
        float joyY = *context.rightThumbstickY;
        if (std::abs(joyY) > 0.1f) { // Deadzone
            // --- NEW: Non-linear speed calculation ---
            const float BASE_JOYSTICK_SPEED = 0.01f;   // Speed when the point is at its base distance
            const float ACCELERATION_FACTOR = 0.05f;   // How quickly speed increases with distance
            const float MAX_DISTANCE_OFFSET = 100.0f;  // A far-away limit (100m)
            const float VISUAL_DISTANCE = 0.2f;
            const float MIN_DISTANCE_OFFSET = 0.01f - VISUAL_DISTANCE; // ~ -0.19m

            // Speed is proportional to the current offset from the base distance
            float dynamicSpeed = BASE_JOYSTICK_SPEED + std::abs(vrDragDistanceOffset) * ACCELERATION_FACTOR;

            vrDragDistanceOffset += joyY * dynamicSpeed;

            // Clamp the offset to prevent it from going too close or too far
            vrDragDistanceOffset = std::clamp(vrDragDistanceOffset, MIN_DISTANCE_OFFSET, MAX_DISTANCE_OFFSET);
        }
    }

    if (isVrTriggerDown) {
        // Check if we should transition to dragging state
        if (!isVrDragging) {
            const uint32_t VR_DRAG_DELAY_MS = 150;
            if (SDL_GetTicks() - vrTriggerDownTimestamp > VR_DRAG_DELAY_MS) {
                isVrDragging = true;
                // The start point is already correctly set from OnLeftMouseDown
            }
        }
        
        // --- CORRECTED LOGIC: Always update only the end point while dragging or preparing to drag ---
        const float VISUAL_DISTANCE = 0.2f;
        float worldScale = context.worldTransform ? glm::length(glm::vec3((*context.worldTransform)[0])) : 1.0f;
        float worldDistance = (VISUAL_DISTANCE + vrDragDistanceOffset) * worldScale;

        // The check for positive distance is now implicitly handled by clamping the offset above
        
        // If we are NOT dragging yet, the end point just follows the ghost point
        if (!isVrDragging) {
            vrDragStartPoint = rayOrigin + rayDirection * worldDistance;
        }
        // Once we ARE dragging, vrDragStartPoint is fixed, and only vrDragEndPoint moves.
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

void SelectTool::GetVrDragBoxCorners(glm::vec3& outStart, glm::vec3& outEnd) const {
    outStart = vrDragStartPoint;
    outEnd = vrDragEndPoint;
}

float SelectTool::GetVrDragDistanceOffset() const {
    return vrDragDistanceOffset;
}

} // namespace Urbaxio::Tools 