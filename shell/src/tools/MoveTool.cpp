#define GLM_ENABLE_EXPERIMENTAL
#include "tools/MoveTool.h"
#include "engine/scene.h"
#include "engine/commands/MoveCommand.h"
#include "camera.h" // For SnappingSystem::WorldToScreen
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

namespace { // Anonymous namespace for helpers
    
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
}

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
    movingObjectId = 0;
    currentTranslation = glm::vec3(0.0f);
    lockedAxisType = SnapType::NONE;
    inferenceAxisDir = glm::vec3(0.0f);
    lengthInputBuf[0] = '\0';
}

void MoveTool::OnLeftMouseDown(int mouseX, int mouseY, bool shift, bool ctrl) {
    if (currentState == MoveToolState::IDLE) {
        if (*context.selectedObjId != 0) {
            movingObjectId = *context.selectedObjId;
            basePoint = lastSnapResult.worldPoint;
            currentState = MoveToolState::AWAITING_DESTINATION_FREE;
            lengthInputBuf[0] = '\0';
            std::cout << "MoveTool: Started moving object " << movingObjectId << " from base point." << std::endl;
        } else {
            std::cout << "MoveTool: Select an object before using the Move tool." << std::endl;
        }
    } else {
        finalizeMove();
    }
}

void MoveTool::OnRightMouseDown() {
    if (currentState != MoveToolState::IDLE) {
        reset();
    }
}

void MoveTool::OnKeyDown(SDL_Keycode key, bool shift, bool ctrl) {
    if (key == SDLK_ESCAPE) {
        reset();
        return;
    }

    if (currentState != MoveToolState::IDLE) {
        if (key == SDLK_RETURN || key == SDLK_KP_ENTER) {
            float length_mm;
            auto [ptr, ec] = std::from_chars(lengthInputBuf, lengthInputBuf + strlen(lengthInputBuf), length_mm);
            if (ec == std::errc() && ptr == lengthInputBuf + strlen(lengthInputBuf)) {
                float length_m = length_mm / 1000.0f;
                if (length_m > 1e-4f) {
                    glm::vec3 direction;
                    if (currentState == MoveToolState::AWAITING_DESTINATION_AXIS_LOCKED) {
                        float dotProd = glm::dot(currentTranslation, lockedAxisDir);
                        direction = (dotProd >= 0.0f) ? lockedAxisDir : -lockedAxisDir;
                    } else if (currentState == MoveToolState::AWAITING_DESTINATION_INFERENCE_LOCKED) {
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
    if (currentState == MoveToolState::IDLE) return;
    
    const Uint8* keyboardState = SDL_GetKeyboardState(NULL);
    bool shiftDown = keyboardState[SDL_SCANCODE_LSHIFT] || keyboardState[SDL_SCANCODE_RSHIFT];

    switch (currentState) {
        case MoveToolState::AWAITING_DESTINATION_FREE:
            if (shiftDown) {
                if (snap.snapped && isValidGeometricSnap(snap.type)) {
                    glm::vec3 direction_to_lock = snap.worldPoint - basePoint;
                    if (glm::length2(direction_to_lock) > 1e-8f) {
                        inferenceAxisDir = glm::normalize(direction_to_lock);
                        currentState = MoveToolState::AWAITING_DESTINATION_INFERENCE_LOCKED;
                        return;
                    }
                }
                if (tryToLockAxis(snap.worldPoint)) {
                    currentState = MoveToolState::AWAITING_DESTINATION_AXIS_LOCKED;
                    return;
                }
            }
            currentTranslation = snap.worldPoint - basePoint;
            break;
        
        case MoveToolState::AWAITING_DESTINATION_AXIS_LOCKED:
            if (!shiftDown) {
                currentState = MoveToolState::AWAITING_DESTINATION_FREE;
                return;
            }
            currentTranslation = calculateAxisLockedPoint(snap) - basePoint;
            break;

        case MoveToolState::AWAITING_DESTINATION_INFERENCE_LOCKED:
            if (!shiftDown) {
                currentState = MoveToolState::AWAITING_DESTINATION_FREE;
                return;
            }
            currentTranslation = calculateInferenceLockedPoint(snap) - basePoint;
            break;
        default:
             currentTranslation = snap.worldPoint - basePoint;
             break;
    }
}

void MoveTool::finalizeMove() {
    if (movingObjectId != 0 && glm::length2(currentTranslation) > 1e-8f) {
        auto command = std::make_unique<Engine::MoveCommand>(
            context.scene,
            movingObjectId,
            currentTranslation
        );
        context.scene->getCommandManager()->ExecuteCommand(std::move(command));
    }
    reset();
}

bool MoveTool::IsMoving() const {
    return currentState != MoveToolState::IDLE;
}

uint64_t MoveTool::GetMovingObjectId() const {
    return movingObjectId;
}

glm::mat4 MoveTool::GetPreviewTransform() const {
    return glm::translate(glm::mat4(1.0f), currentTranslation);
}

void MoveTool::RenderUI() {
    if (currentState != MoveToolState::IDLE) {
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
        float maxDot = -1.0f;
        SnapType bestAxis = SnapType::NONE;
        glm::vec3 bestDir;
        
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
                        if (d > maxDot) { maxDot = d; bestAxis = ax.first; bestDir = ax.second;}
                    }
                }
            }
        }
        
        if(bestAxis != SnapType::NONE) {
            lockedAxisType = bestAxis; lockedAxisDir = bestDir; return true;
        }
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
    if (lockedAxisType == SnapType::AXIS_X || lockedAxisType == SnapType::AXIS_Y) {
        glm::vec3 pointOnGround;
        if (SnappingSystem::RaycastToZPlane(mouseX, mouseY, *context.display_w, *context.display_h, *context.camera, pointOnGround)) {
            return ClosestPointOnLine(basePoint, lockedAxisDir, pointOnGround);
        }
    } else if (lockedAxisType == SnapType::AXIS_Z) {
        glm::vec3 planeOrigin = basePoint;
        glm::vec3 planeNormal = glm::cross(AXIS_Z_DIR, context.camera->Right);
        float denominator = glm::dot(rayDir, planeNormal);
        if (std::abs(denominator) > 1e-6) {
            float t = glm::dot(planeOrigin - rayOrigin, planeNormal) / denominator;
            if (t > 0) {
                glm::vec3 intersectionPoint = rayOrigin + t * rayDir;
                return ClosestPointOnLine(basePoint, lockedAxisDir, intersectionPoint);
            }
        }
    }
    return basePoint + currentTranslation;
}

} // namespace Urbaxio::Tools