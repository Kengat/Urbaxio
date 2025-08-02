#define GLM_ENABLE_EXPERIMENTAL
#include "tools/MoveTool.h"
#include "engine/scene.h"
#include "engine/scene_object.h"
#include "engine/commands/MoveCommand.h"
#include "camera.h"
#include "snapping.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/norm.hpp>
#include <imgui.h>
#include <SDL2/SDL_keyboard.h>
#include <SDL2/SDL_mouse.h>
#include <charconv>
#include <string>
#include <cstring>
#include <iostream>
#include <memory>
#include <numeric> // for std::iota
#include <limits>
#include <list>
#include <map>
#include <set>
#include <algorithm>
#include "engine/line.h" // For Line struct

namespace { // Anonymous namespace for helpers
    
    // Custom comparator for glm::vec3 to use it as a map key
    struct Vec3Comparator {
        bool operator()(const glm::vec3& a, const glm::vec3& b) const {
            const float TOLERANCE = 1e-4f;
            if (std::abs(a.x - b.x) > TOLERANCE) return a.x < b.x;
            if (std::abs(a.y - b.y) > TOLERANCE) return a.y < b.y;
            if (std::abs(a.z - b.z) > TOLERANCE) return a.z < b.z;
            return false;
        }
    };
    
    // Helper to find all coplanar and adjacent triangles, starting from a given one.
    std::vector<size_t> FindCoplanarAdjacentTriangles(
        const Urbaxio::Engine::SceneObject& object,
        size_t startTriangleBaseIndex)
    {
        const float NORMAL_DOT_TOLERANCE = 0.999f;
        const float PLANE_DIST_TOLERANCE = 1e-4f;
        const auto& mesh = object.get_mesh_buffers();
        if (!object.has_mesh() || startTriangleBaseIndex + 2 >= mesh.indices.size()) { return { startTriangleBaseIndex }; }
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

    glm::vec3 ClosestPointOnLine(const glm::vec3& lineOrigin, const glm::vec3& lineDir, const glm::vec3& point) {
        float t = glm::dot(point - lineOrigin, lineDir);
        return lineOrigin + lineDir * t;
    }
    
    const glm::vec3 AXIS_X_DIR(1.0f, 0.0f, 0.0f);
    const glm::vec3 AXIS_Y_DIR(0.0f, 1.0f, 0.0f);
    const glm::vec3 AXIS_Z_DIR(0.0f, 0.0f, 1.0f);
    const float SCREEN_VECTOR_MIN_LENGTH_SQ = 4.0f;

    bool isValidGeometricSnap(Urbaxio::SnapType type) {
        switch (type) {
            case Urbaxio::SnapType::ENDPOINT:
            case Urbaxio::SnapType::MIDPOINT:
            case Urbaxio::SnapType::ON_EDGE:
            case Urbaxio::SnapType::INTERSECTION:
            case Urbaxio::SnapType::CENTER:
            case Urbaxio::SnapType::ORIGIN:
            case Urbaxio::SnapType::AXIS_X:
            case Urbaxio::SnapType::AXIS_Y:
            case Urbaxio::SnapType::AXIS_Z:
                return true;
            default:
                return false;
        }
    }
} // Anonymous namespace end

namespace Urbaxio::Tools {

void MoveTool::Activate(const ToolContext& context) {
    ITool::Activate(context);
    reset();
}

void MoveTool::Deactivate() {
    reset();
    ITool::Deactivate();
}

void MoveTool::reset() {
    currentState = MoveToolState::IDLE;
    lockState = AxisLockState::FREE;
    currentTarget = {};
    currentTranslation = glm::vec3(0.0f);
    inferenceAxisDir = glm::vec3(0.0f);
    ghostMesh.clear();
    ghostWireframeIndices.clear(); // <-- ДОБАВИТЬ ЭТУ СТРОКУ
    ghostMeshActive = false;
    lengthInputBuf[0] = '\0';
}

void MoveTool::determineTargetFromSelection() {
    currentTarget = {}; // Reset target
    if (*context.selectedObjId != 0 && !context.selectedTriangleIndices->empty()) {
        currentTarget.type = MoveTarget::TargetType::FACE;
        currentTarget.objectId = *context.selectedObjId;
        Engine::SceneObject* obj = context.scene->get_object_by_id(currentTarget.objectId);
        if (obj && obj->has_mesh()) {
            const auto& mesh = obj->get_mesh_buffers();
            for (size_t triBaseIndex : *context.selectedTriangleIndices) {
                currentTarget.movingVertices.insert(mesh.indices[triBaseIndex]);
                currentTarget.movingVertices.insert(mesh.indices[triBaseIndex + 1]);
                currentTarget.movingVertices.insert(mesh.indices[triBaseIndex + 2]);
            }
        }
    } else if (*context.selectedObjId != 0) {
        currentTarget.type = MoveTarget::TargetType::OBJECT;
        currentTarget.objectId = *context.selectedObjId;
        Engine::SceneObject* obj = context.scene->get_object_by_id(currentTarget.objectId);
        if (obj && obj->has_mesh()) {
            size_t vertexCount = obj->get_mesh_buffers().vertices.size() / 3;
            currentTarget.movingVertices.clear();
            std::vector<unsigned int> all_indices(vertexCount);
            std::iota(all_indices.begin(), all_indices.end(), 0);
            currentTarget.movingVertices.insert(all_indices.begin(), all_indices.end());
        }
    }
}

void MoveTool::determineTargetFromPick(int mouseX, int mouseY) {
    reset();

    const SnapResult& snap = lastSnapResult;

    // --- FINAL LOGIC: Explicitly ignore non-geometric snaps for picking a target ---
    if (!snap.snapped || snap.type == SnapType::AXIS_X || snap.type == SnapType::AXIS_Y || snap.type == SnapType::AXIS_Z || snap.type == SnapType::GRID) {
        // Fallback to generic face-pick if no valid geometric snap is available.
        glm::vec3 rayOrigin, rayDir;
        Camera::ScreenToWorldRay(mouseX, mouseY, *context.display_w, *context.display_h, context.camera->GetViewMatrix(), context.camera->GetProjectionMatrix((float)*context.display_w / *context.display_h), rayOrigin, rayDir);
        uint64_t hitObjectId = 0;
        size_t hitTriangleBaseIndex = 0;
        float closestHitDistance = std::numeric_limits<float>::max();
        for (auto* obj : context.scene->get_all_objects()) {
            const auto& name = obj->get_name();
            if (!obj || !obj->has_mesh() || name == "CenterMarker" || name == "UnitCapsuleMarker10m" || name == "UnitCapsuleMarker5m") continue;
            const auto& mesh = obj->get_mesh_buffers();
            for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
                unsigned int i0 = mesh.indices[i], i1 = mesh.indices[i+1], i2 = mesh.indices[i+2];
                glm::vec3 v0(mesh.vertices[i0*3], mesh.vertices[i0*3+1], mesh.vertices[i0*3+2]);
                glm::vec3 v1(mesh.vertices[i1*3], mesh.vertices[i1*3+1], mesh.vertices[i1*3+2]);
                glm::vec3 v2(mesh.vertices[i2*3], mesh.vertices[i2*3+1], mesh.vertices[i2*3+2]);
                float t;
                if (SnappingSystem::RayTriangleIntersect(rayOrigin, rayDir, v0, v1, v2, t) && t > 0 && t < closestHitDistance) {
                    closestHitDistance = t;
                    hitObjectId = obj->get_id();
                    hitTriangleBaseIndex = i;
                }
            }
        }
        if (hitObjectId != 0) {
            Engine::SceneObject* hitObject = context.scene->get_object_by_id(hitObjectId);
            currentTarget.type = MoveTarget::TargetType::FACE;
            currentTarget.objectId = hitObjectId;
            std::vector<size_t> faceTriangles = FindCoplanarAdjacentTriangles(*hitObject, hitTriangleBaseIndex);
            for(size_t triBaseIndex : faceTriangles) {
                const auto& mesh = hitObject->get_mesh_buffers();
                currentTarget.movingVertices.insert(mesh.indices[triBaseIndex]);
                currentTarget.movingVertices.insert(mesh.indices[triBaseIndex+1]);
                currentTarget.movingVertices.insert(mesh.indices[triBaseIndex+2]);
            }
        }
        return;
    }

    // --- We have a valid geometric snap, determine the target ---
    Engine::SceneObject* snappedObject = nullptr;
    uint64_t snappedLineId = 0;

    if (snap.snappedEntityId != 0) {
        snappedObject = context.scene->get_object_by_id(snap.snappedEntityId);
        if (snappedObject) {
            // It's an object (likely a face snap)
        } else {
            // It's a line, find its parent object
            snappedLineId = snap.snappedEntityId;
            for(auto* obj : context.scene->get_all_objects()) {
                if (obj->boundaryLineIDs.count(snappedLineId)) {
                    snappedObject = obj;
                    break;
                }
            }
        }
    }

    auto findVertexIndex = [&](const Engine::SceneObject& obj, const glm::vec3& pos) -> int {
        const auto& mesh = obj.get_mesh_buffers();
        for (size_t i = 0; i < mesh.vertices.size() / 3; ++i) {
            glm::vec3 v_pos(mesh.vertices[i*3], mesh.vertices[i*3+1], mesh.vertices[i*3+2]);
            if (glm::distance2(v_pos, pos) < 1e-8f) { return static_cast<int>(i); }
        }
        return -1;
    };

    switch (snap.type) {
        case SnapType::ENDPOINT:
        case SnapType::ORIGIN: {
            if (!snappedObject) {
                 for (auto* obj : context.scene->get_all_objects()) {
                    if (!obj || !obj->has_mesh()) continue;
                    int v_idx = findVertexIndex(*obj, snap.worldPoint);
                    if (v_idx != -1) { snappedObject = obj; break; }
                }
            }
            if (snappedObject) {
                int vertexIndex = findVertexIndex(*snappedObject, snap.worldPoint);
                if (vertexIndex != -1) {
                    currentTarget.type = MoveTarget::TargetType::VERTEX;
                    currentTarget.objectId = snappedObject->get_id();
                    currentTarget.movingVertices.insert(vertexIndex);
                }
            }
            break;
        }

        case SnapType::MIDPOINT: // Midpoint now selects the whole edge
        case SnapType::ON_EDGE: {
            if (snappedObject && snappedLineId != 0) {
                const auto& line = context.scene->GetAllLines().at(snappedLineId);
                int idx1 = findVertexIndex(*snappedObject, line.start);
                int idx2 = findVertexIndex(*snappedObject, line.end);
                if (idx1 != -1 && idx2 != -1) {
                    currentTarget.type = MoveTarget::TargetType::EDGE;
                    currentTarget.objectId = snappedObject->get_id();
                    currentTarget.movingVertices.insert(idx1);
                    currentTarget.movingVertices.insert(idx2);
                }
            }
            break;
        }

        case SnapType::ON_FACE: {
            if (snappedObject) {
                glm::vec3 rayOrigin, rayDir;
                Camera::ScreenToWorldRay(mouseX, mouseY, *context.display_w, *context.display_h, context.camera->GetViewMatrix(), context.camera->GetProjectionMatrix((float)*context.display_w / *context.display_h), rayOrigin, rayDir);
                float closestHitDist = std::numeric_limits<float>::max();
                size_t hitTriangle = -1;
                const auto& mesh = snappedObject->get_mesh_buffers();
                for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
                     unsigned int i0 = mesh.indices[i], i1 = mesh.indices[i+1], i2 = mesh.indices[i+2];
                     glm::vec3 v0(mesh.vertices[i0*3], mesh.vertices[i0*3+1], mesh.vertices[i0*3+2]);
                     glm::vec3 v1(mesh.vertices[i1*3], mesh.vertices[i1*3+1], mesh.vertices[i1*3+2]);
                     glm::vec3 v2(mesh.vertices[i2*3], mesh.vertices[i2*3+1], mesh.vertices[i2*3+2]);
                     float t;
                     if (SnappingSystem::RayTriangleIntersect(rayOrigin, rayDir, v0, v1, v2, t) && t > 0 && t < closestHitDist) {
                         closestHitDist = t;
                         hitTriangle = i;
                     }
                }
                if (hitTriangle != (size_t)-1) {
                    currentTarget.type = MoveTarget::TargetType::FACE;
                    currentTarget.objectId = snappedObject->get_id();
                    std::vector<size_t> faceTriangles = FindCoplanarAdjacentTriangles(*snappedObject, hitTriangle);
                    for(size_t triBaseIndex : faceTriangles) {
                        currentTarget.movingVertices.insert(mesh.indices[triBaseIndex]);
                        currentTarget.movingVertices.insert(mesh.indices[triBaseIndex+1]);
                        currentTarget.movingVertices.insert(mesh.indices[triBaseIndex+2]);
                    }
                }
            }
            break;
        }
        default: break;
    }
}


void MoveTool::startMove(const SnapResult& snap) {
    if (currentTarget.type == MoveTarget::TargetType::NONE) return;

    Engine::SceneObject* obj = context.scene->get_object_by_id(currentTarget.objectId);
    if (!obj || !obj->has_mesh()) return;

    const auto& originalMesh = obj->get_mesh_buffers();

    // --- NEW "STICKY" LOGIC ---
    std::map<glm::vec3, std::vector<unsigned int>, Vec3Comparator> positionToIndices;
    for (size_t i = 0; i < originalMesh.vertices.size() / 3; ++i) {
        glm::vec3 pos(originalMesh.vertices[i*3], originalMesh.vertices[i*3+1], originalMesh.vertices[i*3+2]);
        positionToIndices[pos].push_back(i);
    }
    std::set<glm::vec3, Vec3Comparator> movingPositions;
    for (unsigned int v_idx : currentTarget.movingVertices) {
        glm::vec3 pos(originalMesh.vertices[v_idx*3], originalMesh.vertices[v_idx*3+1], originalMesh.vertices[v_idx*3+2]);
        movingPositions.insert(pos);
    }
    std::set<unsigned int> expandedMovingVertices;
    for (const auto& pos : movingPositions) {
        const auto& indices = positionToIndices.at(pos);
        expandedMovingVertices.insert(indices.begin(), indices.end());
    }
    currentTarget.movingVertices = expandedMovingVertices;
    // --- END STICKY LOGIC ---

    basePoint = snap.worldPoint;
    currentState = MoveToolState::MOVING_PREVIEW;
    lockState = AxisLockState::FREE;
    lengthInputBuf[0] = '\0';
    
    ghostMeshActive = true;
    ghostMesh = originalMesh; // Full copy for deformation
    
    // --- Build the ghost wireframe indices ---
    ghostWireframeIndices.clear();
    // 1. Build a reverse map from position to vertex index
    std::map<glm::vec3, unsigned int, Vec3Comparator> positionToIndexMap;
    for (size_t i = 0; i < originalMesh.vertices.size() / 3; ++i) {
        glm::vec3 pos(originalMesh.vertices[i*3], originalMesh.vertices[i*3+1], originalMesh.vertices[i*3+2]);
        if (positionToIndexMap.find(pos) == positionToIndexMap.end()) {
            positionToIndexMap[pos] = i;
        }
    }
    // 2. Look up line endpoints and add indices to the list
    const auto& allLines = context.scene->GetAllLines();
    for (uint64_t lineId : obj->boundaryLineIDs) {
        auto it = allLines.find(lineId);
        if (it != allLines.end()) {
            const auto& line = it->second;
            auto it_start = positionToIndexMap.find(line.start);
            auto it_end = positionToIndexMap.find(line.end);
            if (it_start != positionToIndexMap.end() && it_end != positionToIndexMap.end()) {
                ghostWireframeIndices.push_back(it_start->second);
                ghostWireframeIndices.push_back(it_end->second);
            }
        }
    }

    std::cout << "MoveTool: Started moving." << std::endl;
}

void MoveTool::OnLeftMouseDown(int mouseX, int mouseY, bool shift, bool ctrl) {
    if (currentState == MoveToolState::IDLE) {
        determineTargetFromSelection();
        if (currentTarget.type == MoveTarget::TargetType::NONE) {
            determineTargetFromPick(mouseX, mouseY);
        }
        startMove(lastSnapResult);
    } else {
        finalizeMove();
    }
}

void MoveTool::OnRightMouseDown() {
    if (currentState == MoveToolState::MOVING_PREVIEW) {
        reset();
    }
}

void MoveTool::OnKeyDown(SDL_Keycode key, bool shift, bool ctrl) {
    if (key == SDLK_ESCAPE) {
        reset();
        return;
    }

    if (currentState == MoveToolState::MOVING_PREVIEW) {
        if (key == SDLK_RETURN || key == SDLK_KP_ENTER) {
            float length_mm;
            auto result = std::from_chars(lengthInputBuf, lengthInputBuf + strlen(lengthInputBuf), length_mm);
            if (result.ec == std::errc() && result.ptr == lengthInputBuf + strlen(lengthInputBuf)) {
                float length_m = length_mm / 1000.0f;
                if (length_m > 1e-4f) {
                    glm::vec3 direction;
                    if (lockState == AxisLockState::AXIS_LOCKED) {
                        float dotProd = glm::dot(currentTranslation, lockedAxisDir);
                        direction = (dotProd >= 0.0f) ? lockedAxisDir : -lockedAxisDir;
                    } else if (lockState == AxisLockState::INFERENCE_LOCKED) {
                        float dotProd = glm::dot(currentTranslation, inferenceAxisDir);
                        direction = (dotProd >= 0.0f) ? inferenceAxisDir : -inferenceAxisDir;
                    } else {
                        if (glm::length2(currentTranslation) < 1e-8f) { reset(); return; }
                        direction = glm::normalize(currentTranslation);
                    }
                    currentTranslation = direction * length_m;
                    finalizeMove();
                }
            } else {
                lengthInputBuf[0] = '\0'; // Clear bad input
            }
        } else if (key == SDLK_BACKSPACE) {
            size_t len = strlen(lengthInputBuf); if (len > 0) lengthInputBuf[len - 1] = '\0';
        } else {
            char c = '\0';
            if ((key >= SDLK_0 && key <= SDLK_9)) c = (char)key;
            else if ((key >= SDLK_KP_0 && key <= SDLK_KP_9)) c = '0' + (key - SDLK_KP_0);
            else if (key == SDLK_PERIOD || key == SDLK_KP_PERIOD) { if (strchr(lengthInputBuf, '.') == nullptr) c = '.'; }
            if (c != '\0') { size_t len = strlen(lengthInputBuf); if (len + 1 < 64) { lengthInputBuf[len] = c; lengthInputBuf[len+1] = '\0'; } }
        }
    }
}

void MoveTool::OnUpdate(const SnapResult& snap) {
    lastSnapResult = snap;
    if (currentState != MoveToolState::MOVING_PREVIEW) return;
    
    const Uint8* keyboardState = SDL_GetKeyboardState(NULL);
    bool shiftDown = keyboardState[SDL_SCANCODE_LSHIFT] || keyboardState[SDL_SCANCODE_RSHIFT];

    glm::vec3 endPoint;

    if (shiftDown && lockState == AxisLockState::FREE) {
        if (snap.snapped && isValidGeometricSnap(snap.type)) {
            glm::vec3 direction_to_lock = snap.worldPoint - basePoint;
            if (glm::length2(direction_to_lock) > 1e-8f) {
                inferenceAxisDir = glm::normalize(direction_to_lock);
                lockState = AxisLockState::INFERENCE_LOCKED;
            }
        }
        else if (tryToLockAxis(snap.worldPoint)) {
            lockState = AxisLockState::AXIS_LOCKED;
        }
    } else if (!shiftDown && lockState != AxisLockState::FREE) {
        lockState = AxisLockState::FREE;
    }

    switch (lockState) {
        case AxisLockState::FREE:
            endPoint = snap.worldPoint;
            break;
        case AxisLockState::AXIS_LOCKED:
            endPoint = calculateAxisLockedPoint(snap);
            break;
        case AxisLockState::INFERENCE_LOCKED:
            endPoint = calculateInferenceLockedPoint(snap);
            break;
    }

    currentTranslation = endPoint - basePoint;
    updateGhostMeshDeformation();
}

void MoveTool::updateGhostMeshDeformation() {
    if (!ghostMeshActive) return;

    Engine::SceneObject* obj = context.scene->get_object_by_id(currentTarget.objectId);
    if (!obj || !obj->has_mesh()) return;

    // This is not the most efficient way, but it's correct and simple for now.
    // It re-copies the original mesh and applies the total translation.
    ghostMesh = obj->get_mesh_buffers();

    for (unsigned int v_idx : currentTarget.movingVertices) {
        if (v_idx * 3 + 2 < ghostMesh.vertices.size()) {
            ghostMesh.vertices[v_idx * 3 + 0] += currentTranslation.x;
            ghostMesh.vertices[v_idx * 3 + 1] += currentTranslation.y;
            ghostMesh.vertices[v_idx * 3 + 2] += currentTranslation.z;
        }
    }
    // The wireframe indices do not need to be updated, as they point to the
    // same vertices that are being deformed in the shared vertex buffer.
}

void MoveTool::finalizeMove() {
    if (currentTarget.objectId != 0 && glm::length2(currentTranslation) > 1e-8f) {
        if (currentTarget.type == MoveTarget::TargetType::OBJECT) {
            auto command = std::make_unique<Engine::MoveCommand>(
                context.scene,
                currentTarget.objectId,
                currentTranslation
            );
            context.scene->getCommandManager()->ExecuteCommand(std::move(command));
        } else {
            std::cout << "MoveTool: Sub-object B-Rep modification is not yet implemented. Preview only." << std::endl;
        }
    }
    reset();
}

bool MoveTool::IsMoving() const {
    return currentState == MoveToolState::MOVING_PREVIEW;
}

uint64_t MoveTool::GetMovingObjectId() const {
    return (currentState == MoveToolState::MOVING_PREVIEW) ? currentTarget.objectId : 0;
}

const CadKernel::MeshBuffers* MoveTool::GetGhostMesh() const {
    return ghostMeshActive ? &ghostMesh : nullptr;
}

const std::vector<unsigned int>* MoveTool::GetGhostWireframeIndices() const {
    return ghostMeshActive ? &ghostWireframeIndices : nullptr;
}

void MoveTool::RenderUI() {
    if (currentState == MoveToolState::MOVING_PREVIEW) {
        ImGui::Separator();
        float length_mm = glm::length(currentTranslation) * 1000.0f;
        ImGui::Text("Distance (mm): %.2f", length_mm);
        ImGui::Text("Input: %s", lengthInputBuf);
        ImGui::Separator();
    }
}

bool MoveTool::tryToLockAxis(const glm::vec3& currentTarget) {
    glm::mat4 view = context.camera->GetViewMatrix();
    glm::mat4 proj = context.camera->GetProjectionMatrix((float)*context.display_w / *context.display_h);
    glm::vec2 startScreenPos, endScreenPos;

    bool sVis = SnappingSystem::WorldToScreen(basePoint, view, proj, *context.display_w, *context.display_h, startScreenPos);
    bool eVis = SnappingSystem::WorldToScreen(currentTarget, view, proj, *context.display_w, *context.display_h, endScreenPos);

    if (sVis && eVis && glm::length2(endScreenPos - startScreenPos) > SCREEN_VECTOR_MIN_LENGTH_SQ) {
        glm::vec2 rbDir = glm::normalize(endScreenPos - startScreenPos);
        float maxDot = 0.8f; // Require a strong alignment
        SnapType bestAxis = SnapType::NONE;
        
        const std::vector<std::pair<SnapType, glm::vec3>> axes = {
            {SnapType::AXIS_X, AXIS_X_DIR}, {SnapType::AXIS_Y, AXIS_Y_DIR}, {SnapType::AXIS_Z, AXIS_Z_DIR}
        };

        glm::vec2 originScreen;
        if(SnappingSystem::WorldToScreen(basePoint, view, proj, *context.display_w, *context.display_h, originScreen)) {
            for(const auto& ax : axes) {
                glm::vec2 axisEndPointScreen;
                if(SnappingSystem::WorldToScreen(basePoint + ax.second, view, proj, *context.display_w, *context.display_h, axisEndPointScreen)) {
                    if (glm::length2(axisEndPointScreen - originScreen) > 1e-6) {
                        glm::vec2 axDir = glm::normalize(axisEndPointScreen - originScreen);
                        float d = abs(glm::dot(rbDir, axDir));
                        if (d > maxDot) { maxDot = d; bestAxis = ax.first; lockedAxisDir = ax.second;}
                    }
                }
            }
        }
        
        return bestAxis != SnapType::NONE;
    }
    return false;
}

glm::vec3 MoveTool::calculateInferenceLockedPoint(const SnapResult& snap) {
    const glm::vec3& axisOrigin = basePoint;
    const glm::vec3& axisDir = inferenceAxisDir; 
    if (snap.snapped && isValidGeometricSnap(snap.type)) {
        return ClosestPointOnLine(axisOrigin, axisDir, snap.worldPoint);
    }
    int mouseX, mouseY; SDL_GetMouseState(&mouseX, &mouseY);
    glm::vec3 rayOrigin, rayDir;
    Camera::ScreenToWorldRay(mouseX, mouseY, *context.display_w, *context.display_h, context.camera->GetViewMatrix(), context.camera->GetProjectionMatrix((float)*context.display_w / *context.display_h), rayOrigin, rayDir);
    glm::vec3 w0 = axisOrigin - rayOrigin;
    float b = glm::dot(axisDir, rayDir);
    float d = glm::dot(axisDir, w0);
    float e = glm::dot(rayDir, w0);
    float denom = 1.0f - b * b;
    if (std::abs(denom) > 1e-6f) {
        float s = (b * e - d) / denom;
        return axisOrigin + s * axisDir;
    }
    return basePoint + currentTranslation;
}

glm::vec3 MoveTool::calculateAxisLockedPoint(const SnapResult& snap) {
    if (snap.snapped && isValidGeometricSnap(snap.type)) {
        return ClosestPointOnLine(basePoint, lockedAxisDir, snap.worldPoint);
    }
    int mouseX, mouseY; SDL_GetMouseState(&mouseX, &mouseY);
    glm::vec3 rayOrigin, rayDir;
    Camera::ScreenToWorldRay(mouseX, mouseY, *context.display_w, *context.display_h, context.camera->GetViewMatrix(), context.camera->GetProjectionMatrix((float)*context.display_w / *context.display_h), rayOrigin, rayDir);
    
    glm::vec3 w0 = basePoint - rayOrigin;
    float b = glm::dot(lockedAxisDir, rayDir);
    float d = glm::dot(lockedAxisDir, w0);
    float e = glm::dot(rayDir, w0);
    float denom = 1.0f - b * b;
    if (std::abs(denom) > 1e-6f) {
        float s = (b * e - d) / denom;
        return basePoint + s * lockedAxisDir;
    }
    
    return basePoint + currentTranslation;
}

} // namespace Urbaxio::Tools