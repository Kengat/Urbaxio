#include "tools/LineTool.h"
#include "engine/scene.h"
#include "engine/scene_object.h" // Required for full SceneObject definition
#include "camera.h"
#include "renderer.h"
#include "snapping.h"
#include <imgui.h>
#include <SDL2/SDL_keyboard.h>
#include <SDL2/SDL_mouse.h> // <-- FIX: Added missing include for mouse functions
#include <glm/gtx/norm.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <charconv>
#include <string>
#include <cstring>
#include <iostream>
#include "engine/commands/CreateLineCommand.h" // <-- NEW
#include <memory> // For std::make_unique

namespace { // Anonymous namespace for helpers

// Checks if the snap type indicates a point on an existing line that requires a split.
bool IsLineSplittingSnap(Urbaxio::SnapType type) {
    switch(type) {
        case Urbaxio::SnapType::MIDPOINT:
        case Urbaxio::SnapType::ON_EDGE:
            return true;
        default:
            return false;
    }
}



void AppendCharToBuffer(char* buf, size_t bufSize, char c) {
    size_t len = strlen(buf);
    if (len + 1 < bufSize) {
        buf[len] = c;
        buf[len + 1] = '\0';
    }
}

void RemoveLastChar(char* buf) {
    size_t len = strlen(buf);
    if (len > 0) {
        buf[len - 1] = '\0';
    }
}

glm::vec3 ClosestPointOnLine(const glm::vec3& lineOrigin, const glm::vec3& lineDir, const glm::vec3& point) {
    float t = glm::dot(point - lineOrigin, lineDir);
    return lineOrigin + lineDir * t;
}

const glm::vec3 AXIS_X_DIR(1.0f, 0.0f, 0.0f);
const glm::vec3 AXIS_Y_DIR(0.0f, 1.0f, 0.0f);
const glm::vec3 AXIS_Z_DIR(0.0f, 0.0f, 1.0f);
const float SCREEN_VECTOR_MIN_LENGTH_SQ = 4.0f;

// Checks for snaps that are valid for axis projection.
bool IsProjectablePointSnap(Urbaxio::SnapType type) {
    switch (type) {
        case Urbaxio::SnapType::ENDPOINT:
        case Urbaxio::SnapType::ORIGIN:
        case Urbaxio::SnapType::MIDPOINT:
        case Urbaxio::SnapType::CENTER:
        case Urbaxio::SnapType::INTERSECTION:
            return true;
        default:
            return false;
    }
}

} // end anonymous namespace

namespace Urbaxio::Tools {

void LineTool::Activate(const ToolContext& context) {
    ITool::Activate(context);
    reset();
}

void LineTool::Deactivate() {
    reset();
    ITool::Deactivate();
}

void LineTool::reset() {
    currentState = ToolState::IDLE;
    lockedAxisType = SnapType::NONE;
    inferenceAxisDir = glm::vec3(0.0f);
    lengthInputBuf[0] = '\0';
}

void LineTool::OnLeftMouseDown(int mouseX, int mouseY, bool shift, bool ctrl, const glm::vec3& rayOrigin, const glm::vec3& rayDirection) {
    if (!context.scene) return;

    // Use the latest snap result from OnUpdate
    glm::vec3 clickPoint = currentRubberBandEnd;

    // Handle splitting existing lines on snap
    if (lastSnapResult.snapped && IsLineSplittingSnap(lastSnapResult.type)) {
        clickPoint = context.scene->SplitLineAtPoint(lastSnapResult.snappedEntityId, lastSnapResult.worldPoint);
    }
    
    if (currentState == ToolState::IDLE) {
        currentLineStartPoint = clickPoint;
        currentState = ToolState::AWAITING_SECOND_POINT_FREE;
        lengthInputBuf[0] = '\0';
    } else if (currentState == ToolState::AWAITING_SECOND_POINT_FREE || currentState == ToolState::AWAITING_SECOND_POINT_AXIS_LOCKED || currentState == ToolState::AWAITING_SECOND_POINT_INFERENCE_LOCKED) {
        finalizeLine(clickPoint);
    }
}

void LineTool::OnRightMouseDown() {
    if (currentState == ToolState::AWAITING_SECOND_POINT_AXIS_LOCKED || currentState == ToolState::AWAITING_SECOND_POINT_INFERENCE_LOCKED) {
        currentState = ToolState::AWAITING_SECOND_POINT_FREE;
    } else if (currentState == ToolState::AWAITING_SECOND_POINT_FREE) {
        currentState = ToolState::IDLE;
        lengthInputBuf[0] = '\0';
    } else if (currentState == ToolState::IDLE) {
        reset();
    }
}

void LineTool::OnKeyDown(SDL_Keycode key, bool shift, bool ctrl) {
    if (key == SDLK_ESCAPE) {
        OnRightMouseDown();
        return;
    }
    
    if (currentState == ToolState::AWAITING_SECOND_POINT_FREE || currentState == ToolState::AWAITING_SECOND_POINT_AXIS_LOCKED || currentState == ToolState::AWAITING_SECOND_POINT_INFERENCE_LOCKED) {
        bool isEnter = (key == SDLK_RETURN || key == SDLK_KP_ENTER);
        bool isBackspace = (key == SDLK_BACKSPACE);

        if (isEnter) {
            float length_mm;
            auto [ptr, ec] = std::from_chars(lengthInputBuf, lengthInputBuf + strlen(lengthInputBuf), length_mm);
            if (ec == std::errc() && ptr == lengthInputBuf + strlen(lengthInputBuf)) {
                float length_m = length_mm / 1000.0f; // Convert mm to meters
                if (length_m > 1e-4f) {
                glm::vec3 direction;
                if (currentState == ToolState::AWAITING_SECOND_POINT_AXIS_LOCKED) {
                    float dotProd = glm::dot(currentRubberBandEnd - currentLineStartPoint, lockedAxisDir);
                    direction = (dotProd >= 0.0f) ? lockedAxisDir : -lockedAxisDir;
                } else if (currentState == ToolState::AWAITING_SECOND_POINT_INFERENCE_LOCKED) {
                    float dotProd = glm::dot(currentRubberBandEnd - currentLineStartPoint, inferenceAxisDir);
                    direction = (dotProd >= 0.0f) ? inferenceAxisDir : -inferenceAxisDir;
                } else {
                    direction = currentRubberBandEnd - currentLineStartPoint;
                    if (glm::length(direction) < 1e-6f) { reset(); return; }
                    direction = glm::normalize(direction);
                }
                glm::vec3 finalEndPoint = currentLineStartPoint + direction * length_m;
                finalizeLine(finalEndPoint);
            }
        } else {
            lengthInputBuf[0] = '\0'; // Clear bad input
        }
        } else if (isBackspace) {
            RemoveLastChar(lengthInputBuf);
        } else {
            char c = '\0';
            if ((key >= SDLK_0 && key <= SDLK_9)) c = (char)key;
            else if ((key >= SDLK_KP_0 && key <= SDLK_KP_9)) c = '0' + (key - SDLK_KP_0);
            else if (key == SDLK_PERIOD || key == SDLK_KP_PERIOD) { if (strchr(lengthInputBuf, '.') == nullptr) c = '.'; }
            if (c != '\0') AppendCharToBuffer(lengthInputBuf, 64, c);
        }
    }
}

void LineTool::OnUpdate(const SnapResult& snap, const glm::vec3& rayOrigin, const glm::vec3& rayDirection) {
    lastSnapResult = snap; 
    const Uint8* keyboardState = SDL_GetKeyboardState(NULL);
    bool shiftDown = keyboardState[SDL_SCANCODE_LSHIFT] || keyboardState[SDL_SCANCODE_RSHIFT];

    glm::vec3 currentTarget = snap.worldPoint;

    switch (currentState) {
        case ToolState::IDLE:
            currentRubberBandEnd = snap.worldPoint;
            break;

        case ToolState::AWAITING_SECOND_POINT_FREE:
            if (shiftDown) {
                // Priority 1: Lock to direction towards a specific geometric snap point ("the sun")
                if (lastSnapResult.snapped && isValidGeometricSnap(lastSnapResult.type)) {
                    glm::vec3 direction_to_lock = lastSnapResult.worldPoint - currentLineStartPoint;
                    if (glm::length2(direction_to_lock) > 1e-8f) { // Ensure it's not the same point
                        inferenceAxisDir = glm::normalize(direction_to_lock);
                        currentState = ToolState::AWAITING_SECOND_POINT_INFERENCE_LOCKED;
                        // Defer point calculation to next frame in new state, so it will immediately use the new axis
                        return; 
                    }
                }
                
                // Priority 2: Fallback to screen-space axis lock if no good snap
                if (tryToLockAxis(currentTarget)) {
                    currentState = ToolState::AWAITING_SECOND_POINT_AXIS_LOCKED;
                    return;
                }
            }
            // If no lock is activated, stay in free mode
            currentRubberBandEnd = currentTarget;
            break;

        case ToolState::AWAITING_SECOND_POINT_AXIS_LOCKED:
            if (!shiftDown) {
                currentState = ToolState::AWAITING_SECOND_POINT_FREE;
                currentRubberBandEnd = snap.worldPoint; // Update immediately
                return;
            }
            currentRubberBandEnd = calculateAxisLockedPoint(snap);
            break;
            
        case ToolState::AWAITING_SECOND_POINT_INFERENCE_LOCKED:
            if (!shiftDown) {
                currentState = ToolState::AWAITING_SECOND_POINT_FREE;
                currentRubberBandEnd = snap.worldPoint; // Update immediately
                return;
            }
            currentRubberBandEnd = calculateInferenceLockedPoint(snap);
            break;
    }
}

void LineTool::finalizeLine(const glm::vec3& endPoint) {
    if (glm::distance2(currentLineStartPoint, endPoint) > 1e-6f) {
        auto command = std::make_unique<Engine::CreateLineCommand>(
            context.scene, 
            currentLineStartPoint, 
            endPoint
        );
        context.scene->getCommandManager()->ExecuteCommand(std::move(command));
    }
    // Reset for the next line
    currentState = ToolState::IDLE;
    lockedAxisType = SnapType::NONE;
    inferenceAxisDir = glm::vec3(0.0f);
    lengthInputBuf[0] = '\0';
}

void LineTool::RenderUI() {
    if (currentState != ToolState::IDLE) {
        ImGui::Separator();
        ImGui::Text("Length (mm): %s", lengthInputBuf);
        ImGui::Separator();
    }
}

void LineTool::RenderPreview(Renderer& renderer, const SnapResult& snap) {
    if (currentState != ToolState::IDLE) {
        renderer.UpdatePreviewLine(currentLineStartPoint, currentRubberBandEnd);
    } else {
        renderer.UpdatePreviewLine(glm::vec3(0.0f), glm::vec3(0.0f), false);
    }
}

// --- NEW/MODIFIED HELPER METHODS ---

bool LineTool::isValidGeometricSnap(SnapType type) {
    // A valid geometric snap is ANY snap that isn't NONE, GRID or ON_FACE
    switch (type) {
        case SnapType::ENDPOINT:
        case SnapType::MIDPOINT:
        case SnapType::ON_EDGE:
        case SnapType::INTERSECTION:
        case SnapType::CENTER:
        case SnapType::ORIGIN:
        case SnapType::AXIS_X:
        case SnapType::AXIS_Y:
        case SnapType::AXIS_Z:
            return true;
        default:
            return false;
    }
}

bool LineTool::tryToLockAxis(const glm::vec3& currentTarget) {
    glm::mat4 view = context.camera->GetViewMatrix();
    glm::mat4 proj = context.camera->GetProjectionMatrix((float)*context.display_w / (float)*context.display_h);
    glm::vec2 startScreenPos, endScreenPos;

    bool sVis = SnappingSystem::WorldToScreen(currentLineStartPoint, view, proj, *context.display_w, *context.display_h, startScreenPos);
    bool eVis = SnappingSystem::WorldToScreen(currentTarget, view, proj, *context.display_w, *context.display_h, endScreenPos);

    if (sVis && eVis && glm::length2(endScreenPos - startScreenPos) > SCREEN_VECTOR_MIN_LENGTH_SQ) {
        glm::vec2 rbDir = glm::normalize(endScreenPos - startScreenPos);
        float maxDot = -1.0f;
        SnapType bestAxis = SnapType::NONE;
        glm::vec3 bestDir;
        
        const std::vector<std::pair<SnapType, glm::vec3>> axes = {
            {SnapType::AXIS_X, AXIS_X_DIR},
            {SnapType::AXIS_Y, AXIS_Y_DIR},
            {SnapType::AXIS_Z, AXIS_Z_DIR}
        };

        glm::vec2 originScreen;
        if(SnappingSystem::WorldToScreen(currentLineStartPoint, view, proj, *context.display_w, *context.display_h, originScreen)) {
            for(const auto& ax : axes) {
                glm::vec2 axisEndPointScreen;
                if(SnappingSystem::WorldToScreen(currentLineStartPoint + ax.second, view, proj, *context.display_w, *context.display_h, axisEndPointScreen)) {
                    if (glm::length2(axisEndPointScreen - originScreen) > 1e-6) {
                        glm::vec2 axDir = glm::normalize(axisEndPointScreen - originScreen);
                        float d = abs(glm::dot(rbDir, axDir));
                        if (d > maxDot) { maxDot = d; bestAxis = ax.first; bestDir = ax.second;}
                    }
                }
            }
        }
        
        if(bestAxis != SnapType::NONE) {
            lockedAxisType = bestAxis;
            lockedAxisDir = bestDir;
            return true;
        }
    }
    return false;
}

glm::vec3 LineTool::calculateAxisLockedPoint(const SnapResult& snapResult) {
    // Priority 1: Check for a valid geometric snap from the snapping system.
    if (snapResult.snapped && isValidGeometricSnap(snapResult.type)) {
        return ClosestPointOnLine(currentLineStartPoint, lockedAxisDir, snapResult.worldPoint);
    }
    
    // Priority 2: Fallback to free-floating movement if no suitable snap is active.
    int mouseX, mouseY;
    SDL_GetMouseState(&mouseX, &mouseY);
    glm::vec3 rayOrigin, rayDir;
    Camera::ScreenToWorldRay(mouseX, mouseY, *context.display_w, *context.display_h, context.camera->GetViewMatrix(), context.camera->GetProjectionMatrix((float)*context.display_w / (float)*context.display_h), rayOrigin, rayDir);

    if (lockedAxisType == SnapType::AXIS_X || lockedAxisType == SnapType::AXIS_Y) {
        // "Horizontal" Strategy: Project onto the Z=0 construction plane
        glm::vec3 pointOnGround;
        if (SnappingSystem::RaycastToZPlane(mouseX, mouseY, *context.display_w, *context.display_h, *context.camera, pointOnGround)) {
            return ClosestPointOnLine(currentLineStartPoint, lockedAxisDir, pointOnGround);
        }
    } else if (lockedAxisType == SnapType::AXIS_Z) {
        // "Vertical" Strategy: Project onto the virtual vertical plane
        glm::vec3 planeOrigin = currentLineStartPoint;
        
        glm::vec3 planeNormal = glm::cross(AXIS_Z_DIR, context.camera->Right);

        float denominator = glm::dot(rayDir, planeNormal);
        if (std::abs(denominator) > 1e-6) { // Avoid cases where the ray is parallel to the plane
            float t = glm::dot(planeOrigin - rayOrigin, planeNormal) / denominator;
            if (t > 0) {
                glm::vec3 intersectionPoint = rayOrigin + t * rayDir;
                // The final point must lie on the Z-axis, so we only take the Z coordinate from the intersection.
                return glm::vec3(currentLineStartPoint.x, currentLineStartPoint.y, intersectionPoint.z);
            }
        }
    }
    
    // Fallback if any calculation fails (e.g., parallel ray)
    return currentRubberBandEnd; 
}

glm::vec3 LineTool::calculateInferenceLockedPoint(const SnapResult& snap) {
    // Define the inference axis
    const glm::vec3& axisOrigin = currentLineStartPoint;
    const glm::vec3& axisDir = inferenceAxisDir; 

    // Priority 1: Check for a valid geometric snap from the snapping system.
    if (snap.snapped && isValidGeometricSnap(snap.type)) {
        // Project the snapped point ("Alpha Centauri") onto our line of fire.
        return ClosestPointOnLine(axisOrigin, axisDir, snap.worldPoint);
    }

    // Priority 2: Fallback to free-floating movement if no suitable snap is active.
    int mouseX, mouseY;
    SDL_GetMouseState(&mouseX, &mouseY);
    glm::vec3 rayOrigin, rayDir;
    Camera::ScreenToWorldRay(mouseX, mouseY, *context.display_w, *context.display_h, context.camera->GetViewMatrix(), context.camera->GetProjectionMatrix((float)*context.display_w / (float)*context.display_h), rayOrigin, rayDir);

    glm::vec3 w0 = axisOrigin - rayOrigin;
    float b = glm::dot(axisDir, rayDir);
    float d = glm::dot(axisDir, w0);
    float e = glm::dot(rayDir, w0);
    float denom = 1.0f - b * b;

    if (std::abs(denom) > 1e-6f) {
        float s = (b * e - d) / denom;
        return axisOrigin + s * axisDir;
    }

    // Fallback if lines are parallel
    return currentRubberBandEnd;
}

} // namespace Urbaxio::Tools 