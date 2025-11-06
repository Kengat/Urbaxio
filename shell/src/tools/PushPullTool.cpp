#include "tools/PushPullTool.h"
#include "engine/scene.h"
#include "engine/scene_object.h"
#include "cad_kernel/MeshBuffers.h"
#include "camera.h"
#include "renderer.h"
#include "snapping.h"
#include <imgui.h>
#include <SDL2/SDL_mouse.h>
#include <glm/gtx/intersect.hpp>
#include <glm/gtx/norm.hpp>
#include <charconv>
#include <string>
#include <cstring>
#include <iostream>
#include <limits>
#include <set>
#include <map>
#include <list>
#include "engine/commands/PushPullCommand.h"
#include <memory> // For std::make_unique

namespace { // Anonymous namespace for helpers
    void AppendCharToBuffer(char* buf, size_t bufSize, char c) {
        size_t len = strlen(buf);
        if (len + 1 < bufSize) {
            buf[len] = c;
            buf[len + 1] = '\0';
        }
    }
    void RemoveLastChar(char* buf) {
        size_t len = strlen(buf);
        if (len > 0) { buf[len - 1] = '\0'; }
    }
}

namespace { // Anonymous namespace for helpers

// Re-using FindCoplanarAdjacentTriangles from SelectTool's anonymous namespace is tricky.
// So we just copy the improved version here.
std::vector<size_t> FindCoplanarAdjacentTriangles_ForPushPull(
    const Urbaxio::Engine::SceneObject& object,
    size_t startTriangleBaseIndex)
{
    const auto& name = object.get_name();
    if (name == "CenterMarker" || name == "UnitCapsuleMarker10m" || name == "UnitCapsuleMarker5m") {
        return { startTriangleBaseIndex };
    }
    const float NORMAL_DOT_TOLERANCE = 0.999f;
    const float PLANE_DIST_TOLERANCE = 1e-4f;
    const auto& mesh = object.getMeshBuffers();
    if (!object.hasMesh() || startTriangleBaseIndex + 2 >= mesh.indices.size()) { return { startTriangleBaseIndex }; }
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
    unsigned int i0 = mesh.indices[startTriangleBaseIndex];
    glm::vec3 v0(mesh.vertices[i0*3], mesh.vertices[i0*3+1], mesh.vertices[i0*3+2]);
    glm::vec3 referenceNormal(mesh.normals[i0*3], mesh.normals[i0*3+1], mesh.normals[i0*3+2]);
    float referencePlaneD = -glm::dot(referenceNormal, v0);
    queue.push_back(startTriangleBaseIndex);
    visitedTriangles.insert(startTriangleBaseIndex);
    while (!queue.empty()) {
        size_t currentTriangleIndex = queue.front();
        queue.pop_front();
        resultFaceTriangles.push_back(currentTriangleIndex);
        unsigned int current_v_indices[3] = { mesh.indices[currentTriangleIndex], mesh.indices[currentTriangleIndex + 1], mesh.indices[currentTriangleIndex + 2] };
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
                    if (glm::abs(glm::dot(referenceNormal, neighborNormal)) > NORMAL_DOT_TOLERANCE) {
                        float dist = glm::abs(glm::dot(referenceNormal, n_v0) + referencePlaneD);
                        if (dist < PLANE_DIST_TOLERANCE) queue.push_back(neighborIndex);
                    }
                }
            }
        }
    }
    return resultFaceTriangles;
}

glm::vec3 ClosestPointOnLine_ForPushPull(const glm::vec3& lineOrigin, const glm::vec3& lineDir, const glm::vec3& point) {
    float t = glm::dot(point - lineOrigin, lineDir);
    return lineOrigin + lineDir * t;
}

bool IsValidPushPullSnap(Urbaxio::SnapType type) {
    switch (type) {
        case Urbaxio::SnapType::ENDPOINT:
        case Urbaxio::SnapType::MIDPOINT:
        case Urbaxio::SnapType::ORIGIN:
        case Urbaxio::SnapType::CENTER:
        case Urbaxio::SnapType::AXIS_X:
        case Urbaxio::SnapType::AXIS_Y:
        case Urbaxio::SnapType::AXIS_Z:
            return true;
        default:
            return false;
    }
}

} // end anonymous namespace

namespace Urbaxio::Tools {

void PushPullTool::Activate(const ToolContext& context) {
    ITool::Activate(context);
    reset();
}

void PushPullTool::Deactivate() {
    reset();
    ITool::Deactivate();
}

void PushPullTool::reset() {
    isPushPullActive = false;
    pushPull_objId = 0;
    pushPull_faceIndices.clear();
    pushPullCurrentLength = 0.0f;
    lengthInputBuf[0] = '\0';
    if(context.hoveredObjId) *context.hoveredObjId = 0;
    if(context.hoveredFaceTriangleIndices) context.hoveredFaceTriangleIndices->clear();
}

void PushPullTool::OnLeftMouseDown(int mouseX, int mouseY, bool shift, bool ctrl, const glm::vec3& rayOrigin, const glm::vec3& rayDirection) {
    if (isPushPullActive) {
        finalizePushPull(ctrl);
    } else {
        if (*context.hoveredObjId != 0 && !context.hoveredFaceTriangleIndices->empty()) {
            isPushPullActive = true;
            pushPull_objId = *context.hoveredObjId;
            pushPull_faceIndices = *context.hoveredFaceTriangleIndices;
            
            Urbaxio::Engine::SceneObject* obj = context.scene->get_object_by_id(pushPull_objId);
            if (!obj || !obj->hasMesh()) { reset(); return; }

            const auto& mesh = obj->getMeshBuffers();
            if (mesh.isEmpty()) { reset(); return; }

            std::set<unsigned int> uniqueVertIndices;
            for (size_t baseIdx : pushPull_faceIndices) {
                uniqueVertIndices.insert(mesh.indices[baseIdx]);
                uniqueVertIndices.insert(mesh.indices[baseIdx + 1]);
                uniqueVertIndices.insert(mesh.indices[baseIdx + 2]);
            }
            glm::vec3 averagedNormal(0.0f);
            for (unsigned int vIdx : uniqueVertIndices) {
                averagedNormal.x += mesh.normals[vIdx * 3];
                averagedNormal.y += mesh.normals[vIdx * 3 + 1];
                averagedNormal.z += mesh.normals[vIdx * 3 + 2];
            }
            if (glm::length2(averagedNormal) > 1e-9f) {
                pushPull_faceNormal = glm::normalize(averagedNormal);
            } else {
                unsigned int firstIdx = mesh.indices[pushPull_faceIndices[0]];
                pushPull_faceNormal = glm::normalize(glm::vec3(mesh.normals[firstIdx*3], mesh.normals[firstIdx*3+1], mesh.normals[firstIdx*3+2]));
            }
            
            glm::vec3 currentRayOrigin, currentRayDir;
            if (glm::length2(rayDirection) > 1e-9) {
                currentRayOrigin = rayOrigin;
                currentRayDir = rayDirection;
            } else {
                 Camera::ScreenToWorldRay(mouseX, mouseY, *context.display_w, *context.display_h, context.camera->GetViewMatrix(), context.camera->GetProjectionMatrix((float)*context.display_w/(float)*context.display_h), currentRayOrigin, currentRayDir);
            }
            
            float hitDist;
            glm::vec3 facePoint = glm::vec3(mesh.vertices[mesh.indices[pushPull_faceIndices[0]]*3], mesh.vertices[mesh.indices[pushPull_faceIndices[0]]*3+1], mesh.vertices[mesh.indices[pushPull_faceIndices[0]]*3+2]);
            glm::intersectRayPlane(currentRayOrigin, currentRayDir, facePoint, pushPull_faceNormal, hitDist);
            pushPull_startPoint = currentRayOrigin + currentRayDir * hitDist;
            
            pushPullCurrentLength = 0.0f;
            lengthInputBuf[0] = '\0';

            *context.selectedObjId = 0;
            context.selectedTriangleIndices->clear();
        }
    }
}

void PushPullTool::OnRightMouseDown() {
    if (isPushPullActive) {
        reset();
    }
}

void PushPullTool::OnKeyDown(SDL_Keycode key, bool shift, bool ctrl) {
    if (key == SDLK_ESCAPE) {
        OnRightMouseDown();
        return;
    }
    
    // Handle numeric input when push/pull is active
    if (isPushPullActive) {
        if (key == SDLK_RETURN || key == SDLK_KP_ENTER) {
            float length_mm;
            auto result = std::from_chars(lengthInputBuf, lengthInputBuf + strlen(lengthInputBuf), length_mm);
            if (result.ec == std::errc() && result.ptr == lengthInputBuf + strlen(lengthInputBuf)) {
                // Use the current direction of the pull to determine the sign
                float sign = (pushPullCurrentLength >= 0.0f) ? 1.0f : -1.0f;
                // Apply the entered length (converted from mm to meters) with the correct sign
                pushPullCurrentLength = (std::abs(length_mm) / 1000.0f) * sign;
                finalizePushPull(ctrl);
            } else {
                lengthInputBuf[0] = '\0'; // Clear bad input
            }
        } else if (key == SDLK_BACKSPACE) {
            RemoveLastChar(lengthInputBuf);
        } else {
            char c = '\0';
            if (key >= SDLK_0 && key <= SDLK_9) {
                c = (char)key;
            } else if (key >= SDLK_KP_0 && key <= SDLK_KP_9) {
                c = '0' + (key - SDLK_KP_0);
            } else if (key == SDLK_PERIOD || key == SDLK_KP_PERIOD) {
                // Allow dot only if it's not already there
                if (strchr(lengthInputBuf, '.') == nullptr) {
                    c = '.';
                }
            }
            if (c != '\0') {
                AppendCharToBuffer(lengthInputBuf, sizeof(lengthInputBuf), c);
            }
        }
    }
}

// -- START OF MODIFICATION --

void PushPullTool::OnMouseMove(int mouseX, int mouseY) {

    if (isPushPullActive) return; // Don't update hover while pulling

    glm::vec3 rayOrigin, rayDir;

    Camera::ScreenToWorldRay(mouseX, mouseY, *context.display_w, *context.display_h, context.camera->GetViewMatrix(), context.camera->GetProjectionMatrix((float)*context.display_w / *context.display_h), rayOrigin, rayDir);

    updateHover(rayOrigin, rayDir);

}

// -- END OF MODIFICATION --

// -- START OF MODIFICATION --
void PushPullTool::OnUpdate(const SnapResult& snap, const glm::vec3& rayOrigin, const glm::vec3& rayDirection) {
    // 1. Hover logic (only when not pulling)
    if (!isPushPullActive) {
        if (glm::length2(rayDirection) > 1e-9) { // Check if we got a valid ray (from VR)
            updateHover(rayOrigin, rayDirection);
        }
        return;
    }

    // --- NEW: Freeze preview when numpad is active ---
    if (context.isNumpadActive && *context.isNumpadActive) {
        return;
    }

    // 2. Active pull logic
    // Priority 1: Use a valid geometric snap if available.
    if (snap.snapped && IsValidPushPullSnap(snap.type)) {
        glm::vec3 projectedPoint = ClosestPointOnLine_ForPushPull(pushPull_startPoint, pushPull_faceNormal, snap.worldPoint);

        glm::vec3 offsetVector = projectedPoint - pushPull_startPoint;

        pushPullCurrentLength = glm::dot(offsetVector, pushPull_faceNormal);

        return;
    }

    // Priority 2: Fallback to intersecting the current ray with the extrusion AXIS.
    glm::vec3 currentRayOrigin, currentRayDir;
    if (glm::length2(rayDirection) > 1e-9) { // VR ray was provided
        currentRayOrigin = rayOrigin;
        currentRayDir = rayDirection;
    } else { // 2D mode, get ray from mouse
        int mouseX, mouseY; 
        SDL_GetMouseState(&mouseX, &mouseY);
        Camera::ScreenToWorldRay(mouseX, mouseY, *context.display_w, *context.display_h, context.camera->GetViewMatrix(), context.camera->GetProjectionMatrix((float)*context.display_w/(float)*context.display_h), currentRayOrigin, currentRayDir);
    }

    
    // This is the classic, working logic: find closest point between two lines (ray and axis)
    const glm::vec3& axisOrigin = pushPull_startPoint;
    const glm::vec3& axisDir = pushPull_faceNormal;
    glm::vec3 w0 = axisOrigin - currentRayOrigin;
    float b = glm::dot(axisDir, currentRayDir);
    float d = glm::dot(axisDir, w0);
    float e = glm::dot(w0, currentRayDir);
    float denom = 1.0f - b * b;
    if (std::abs(denom) > 1e-6f) {
        float s = (b * e - d) / denom;
        pushPullCurrentLength = s;
    }
}
// -- END OF MODIFICATION --

void PushPullTool::finalizePushPull(bool ctrl) {
    if (context.scene && pushPull_objId != 0) {
        if (std::abs(pushPullCurrentLength) > 1e-4) {
            
            Urbaxio::Engine::SceneObject* obj = context.scene->get_object_by_id(pushPull_objId);
            if (!obj) { reset(); return; }
            
            const auto& mesh = obj->getMeshBuffers();
            std::set<unsigned int> uniqueVertIndices;
            for (size_t baseIdx : pushPull_faceIndices) {
                uniqueVertIndices.insert(mesh.indices[baseIdx]);
                uniqueVertIndices.insert(mesh.indices[baseIdx + 1]);
                uniqueVertIndices.insert(mesh.indices[baseIdx + 2]);
            }
            
            std::vector<glm::vec3> faceVertices;
            for (unsigned int vIdx : uniqueVertIndices) {
                faceVertices.push_back({
                    mesh.vertices[vIdx * 3],
                    mesh.vertices[vIdx * 3 + 1],
                    mesh.vertices[vIdx * 3 + 2]
                });
            }

            auto command = std::make_unique<Urbaxio::Engine::PushPullCommand>(
                context.scene, 
                faceVertices,
                pushPull_faceNormal,
                pushPullCurrentLength,
                ctrl
            );
            context.scene->getCommandManager()->ExecuteCommand(std::move(command));
        }
    }
    reset();
}

void PushPullTool::updateHover(const glm::vec3& rayOrigin, const glm::vec3& rayDirection) {
    *context.hoveredObjId = 0;
    context.hoveredFaceTriangleIndices->clear();
    
    uint64_t currentHoveredObjId = 0;
    size_t currentHoveredTriangleIdx = 0;
    float closestHitDist = std::numeric_limits<float>::max();

    for (Urbaxio::Engine::SceneObject* obj_ptr : context.scene->get_all_objects()) {
        const auto& name = obj_ptr->get_name();
        if (obj_ptr && obj_ptr->hasMesh() && name != "CenterMarker" && name != "UnitCapsuleMarker10m" && name != "UnitCapsuleMarker5m" && name != "LeftControllerVisual" && name != "RightControllerVisual") {
            const auto& mesh = obj_ptr->getMeshBuffers();
            for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
                glm::vec3 v0(mesh.vertices[mesh.indices[i]*3], mesh.vertices[mesh.indices[i]*3+1], mesh.vertices[mesh.indices[i]*3+2]);
                glm::vec3 v1(mesh.vertices[mesh.indices[i+1]*3], mesh.vertices[mesh.indices[i+1]*3+1], mesh.vertices[mesh.indices[i+1]*3+2]);
                glm::vec3 v2(mesh.vertices[mesh.indices[i+2]*3], mesh.vertices[mesh.indices[i+2]*3+1], mesh.vertices[mesh.indices[i+2]*3+2]);
                float t;
                if (SnappingSystem::RayTriangleIntersect(rayOrigin, rayDirection, v0, v1, v2, t) && t > 0 && t < closestHitDist) {
                    closestHitDist = t;
                    currentHoveredObjId = obj_ptr->get_id();
                    currentHoveredTriangleIdx = i;
                }
            }
        }
    }

    if (currentHoveredObjId != 0) {
        Urbaxio::Engine::SceneObject* hitObject = context.scene->get_object_by_id(currentHoveredObjId);
        if (hitObject) {
            *context.hoveredFaceTriangleIndices = FindCoplanarAdjacentTriangles_ForPushPull(*hitObject, currentHoveredTriangleIdx);
            *context.hoveredObjId = currentHoveredObjId;
        }
    }
}

void PushPullTool::RenderUI() {
    if (isPushPullActive) {
        ImGui::Separator();
        float length_mm = std::abs(pushPullCurrentLength * 1000.0f);
        ImGui::Text("Current Distance (mm): %.2f", length_mm);
        ImGui::Text("Length (mm): %s", lengthInputBuf);
        ImGui::Separator();
    }
}

void PushPullTool::RenderPreview(Renderer& renderer, const SnapResult& snap) {
    if (isPushPullActive) {
        Urbaxio::Engine::SceneObject* obj = context.scene->get_object_by_id(pushPull_objId);
        if (obj) {
            renderer.UpdatePushPullPreview(*obj, pushPull_faceIndices, pushPull_faceNormal, pushPullCurrentLength);
        }
    } else {
        // Clear preview mesh when not active
        renderer.UpdatePushPullPreview(Urbaxio::Engine::SceneObject(0, ""), {}, {}, 0.0f);
    }
}

void PushPullTool::SetLengthInput(const std::string& input) {
    size_t copyLen = std::min(input.length(), sizeof(lengthInputBuf) - 1);
    std::memcpy(lengthInputBuf, input.c_str(), copyLen);
    lengthInputBuf[copyLen] = '\0';
}

float PushPullTool::GetCurrentLength() const {
    return pushPullCurrentLength;
}

} // namespace Urbaxio::Tools 