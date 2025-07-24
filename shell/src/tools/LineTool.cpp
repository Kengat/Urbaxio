#include "tools/LineTool.h"
#include "engine/scene.h"
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
    lineLengthInputBuf[0] = '\0';
}

void LineTool::OnLeftMouseDown(int mouseX, int mouseY, bool shift, bool ctrl) {
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
        lineLengthInputBuf[0] = '\0';
    } else if (currentState == ToolState::AWAITING_SECOND_POINT_FREE || currentState == ToolState::AWAITING_SECOND_POINT_AXIS_LOCKED) {
        finalizeLine(clickPoint);
    }
}

void LineTool::OnRightMouseDown() {
    if (currentState == ToolState::AWAITING_SECOND_POINT_AXIS_LOCKED) {
        currentState = ToolState::AWAITING_SECOND_POINT_FREE;
    } else if (currentState == ToolState::AWAITING_SECOND_POINT_FREE) {
        currentState = ToolState::IDLE;
        lineLengthInputBuf[0] = '\0';
    } else if (currentState == ToolState::IDLE) {
        reset();
    }
}

void LineTool::OnKeyDown(SDL_Keycode key, bool shift, bool ctrl) {
    if (key == SDLK_ESCAPE) {
        OnRightMouseDown();
        return;
    }
    
    if (currentState == ToolState::AWAITING_SECOND_POINT_FREE || currentState == ToolState::AWAITING_SECOND_POINT_AXIS_LOCKED) {
        bool isEnter = (key == SDLK_RETURN || key == SDLK_KP_ENTER);
        bool isBackspace = (key == SDLK_BACKSPACE);

        if (isEnter) {
            float length;
            auto [ptr, ec] = std::from_chars(lineLengthInputBuf, lineLengthInputBuf + strlen(lineLengthInputBuf), length);
            if (ec == std::errc() && ptr == lineLengthInputBuf + strlen(lineLengthInputBuf) && length > 1e-4f) {
                glm::vec3 direction;
                if (currentState == ToolState::AWAITING_SECOND_POINT_AXIS_LOCKED) {
                    float dotProd = glm::dot(currentRubberBandEnd - currentLineStartPoint, lockedAxisDir);
                    direction = (dotProd >= 0.0f) ? lockedAxisDir : -lockedAxisDir;
                } else {
                    direction = currentRubberBandEnd - currentLineStartPoint;
                    if (glm::length(direction) < 1e-6f) { reset(); return; }
                    direction = glm::normalize(direction);
                }
                glm::vec3 finalEndPoint = currentLineStartPoint + direction * length;
                finalizeLine(finalEndPoint);
            } else {
                lineLengthInputBuf[0] = '\0'; // Clear bad input
            }
        } else if (isBackspace) {
            RemoveLastChar(lineLengthInputBuf);
        } else {
            char c = '\0';
            if ((key >= SDLK_0 && key <= SDLK_9)) c = (char)key;
            else if ((key >= SDLK_KP_0 && key <= SDLK_KP_9)) c = '0' + (key - SDLK_KP_0);
            else if (key == SDLK_PERIOD || key == SDLK_KP_PERIOD) { if (strchr(lineLengthInputBuf, '.') == nullptr) c = '.'; }
            if (c != '\0') AppendCharToBuffer(lineLengthInputBuf, 64, c);
        }
    }
}

void LineTool::OnUpdate(const SnapResult& snap) {
    lastSnapResult = snap; 
    const Uint8* keyboardState = SDL_GetKeyboardState(NULL);
    bool shiftDown = keyboardState[SDL_SCANCODE_LSHIFT] || keyboardState[SDL_SCANCODE_RSHIFT];

    switch (currentState) {
        case ToolState::IDLE:
            currentRubberBandEnd = snap.worldPoint;
            break;

        case ToolState::AWAITING_SECOND_POINT_FREE:
            if (shiftDown && tryToLockAxis(snap.worldPoint)) {
                currentState = ToolState::AWAITING_SECOND_POINT_AXIS_LOCKED;
                // Defer point calculation to the next frame in the correct state
                return;
            }
            currentRubberBandEnd = snap.worldPoint;
            break;

        case ToolState::AWAITING_SECOND_POINT_AXIS_LOCKED:
            if (!shiftDown) {
                currentState = ToolState::AWAITING_SECOND_POINT_FREE;
                currentRubberBandEnd = snap.worldPoint; // Update immediately
                return;
            }
            currentRubberBandEnd = calculateAxisLockedPoint(snap);
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
    lineLengthInputBuf[0] = '\0';
}

void LineTool::RenderUI() {
    if (currentState != ToolState::IDLE) {
        ImGui::Separator();
        ImGui::Text("Length: %s", lineLengthInputBuf);
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

bool LineTool::isValidGeometricSnap(SnapType type) {
    // A valid geometric snap is ANY snap that isn't NONE or GRID
    switch (type) {
        case SnapType::ENDPOINT:
        case SnapType::MIDPOINT:
        case SnapType::ON_EDGE:
        case SnapType::ON_FACE:
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

glm::vec3 LineTool::calculateAxisLockedPoint(const SnapResult& snapResult) {
    // Step 0: High-priority geometric snap (inference point)
    if (snapResult.snapped && isValidGeometricSnap(snapResult.type)) {
        return ClosestPointOnLine(currentLineStartPoint, lockedAxisDir, snapResult.worldPoint);
    }
    
    // Step 1 & 2: Hybrid Strategy for empty space
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
        
        // --- FIX: The normal must be perpendicular to the camera's UP direction on the XY plane ---
        // This makes the plane sensitive to vertical mouse movement.
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

} // namespace Urbaxio::Tools 