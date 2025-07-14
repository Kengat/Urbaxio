#include "tools/LineTool.h"
#include "engine/scene.h"
#include "camera.h"
#include "renderer.h"
#include "snapping.h"
#include <imgui.h>
#include <SDL2/SDL_keyboard.h>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <charconv>
#include <string>
#include <cstring>
#include <iostream>

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
    isPlacingFirstPoint = true;
}

void LineTool::Deactivate() {
    reset();
    ITool::Deactivate();
}

void LineTool::reset() {
    isPlacingFirstPoint = false;
    isPlacingSecondPoint = false;
    isAxisLocked = false;
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
    
    if (isPlacingFirstPoint) {
        currentLineStartPoint = clickPoint;
        isPlacingFirstPoint = false;
        isPlacingSecondPoint = true;
        isAxisLocked = false;
        lineLengthInputBuf[0] = '\0';
    } else if (isPlacingSecondPoint) {
        finalizeLine(clickPoint);
    }
}

void LineTool::OnRightMouseDown() {
    if (isAxisLocked) {
        isAxisLocked = false;
    } else if (isPlacingSecondPoint) {
        isPlacingFirstPoint = true;
        isPlacingSecondPoint = false;
        isAxisLocked = false;
        lineLengthInputBuf[0] = '\0';
    } else if (isPlacingFirstPoint) {
        reset(); // Effectively cancels the tool start
    }
}

void LineTool::OnKeyDown(SDL_Keycode key, bool shift, bool ctrl) {
    bool isEnter = (key == SDLK_RETURN || key == SDLK_KP_ENTER);
    bool isBackspace = (key == SDLK_BACKSPACE);

    if (key == SDLK_ESCAPE) {
        OnRightMouseDown(); // Same logic as right-click
        return;
    }
    
    // Shift key is for axis locking, handled in OnUpdate.
    // Length input handling
    if (isPlacingSecondPoint) {
        if (isEnter) {
            float length;
            auto [ptr, ec] = std::from_chars(lineLengthInputBuf, lineLengthInputBuf + strlen(lineLengthInputBuf), length);
            if (ec == std::errc() && ptr == lineLengthInputBuf + strlen(lineLengthInputBuf) && length > 1e-4f) {
                glm::vec3 direction;
                if (isAxisLocked) {
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
    lastSnapResult = snap; // Store the latest snap result

    if (!isPlacingSecondPoint) {
        currentRubberBandEnd = snap.worldPoint;
        return;
    }

    // Axis locking logic (activated with Shift)
    const Uint8* keyboardState = SDL_GetKeyboardState(NULL);
    bool shiftDown = keyboardState[SDL_SCANCODE_LSHIFT] || keyboardState[SDL_SCANCODE_RSHIFT];
    
    if (!shiftDown && isAxisLocked) {
        isAxisLocked = false;
    } else if (shiftDown && !isAxisLocked) {
        glm::mat4 view = context.camera->GetViewMatrix();
        glm::mat4 proj = context.camera->GetProjectionMatrix((float)*context.display_w / (float)*context.display_h);
        glm::vec2 startScreenPos, endScreenPos;
        bool sVis = SnappingSystem::WorldToScreen(currentLineStartPoint, view, proj, *context.display_w, *context.display_h, startScreenPos);
        bool eVis = SnappingSystem::WorldToScreen(snap.worldPoint, view, proj, *context.display_w, *context.display_h, endScreenPos);

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
            glm::vec2 oScreen;
            if(SnappingSystem::WorldToScreen(glm::vec3(0.0f), view, proj, *context.display_w, *context.display_h, oScreen)) {
                for(const auto& ax : axes) {
                    glm::vec2 axScreen;
                    if(SnappingSystem::WorldToScreen(ax.second, view, proj, *context.display_w, *context.display_h, axScreen)) {
                        glm::vec2 axDir = glm::normalize(axScreen - oScreen);
                        float d = abs(glm::dot(rbDir, axDir));
                        if (d > maxDot) { maxDot = d; bestAxis = ax.first; bestDir = ax.second;}
                    }
                }
            }
            if(bestAxis != SnapType::NONE) {
                isAxisLocked = true;
                lockedAxisType = bestAxis;
                lockedAxisDir = bestDir;
            }
        }
    }

    // Update rubber band end point based on snap and axis lock
    if (isAxisLocked) {
        if (snap.snapped && IsProjectablePointSnap(snap.type)) {
            // Project the valid snap point onto the locked axis
            currentRubberBandEnd = ClosestPointOnLine(currentLineStartPoint, lockedAxisDir, snap.worldPoint);
        } else {
            // Project the raw cursor world point onto the locked axis
            currentRubberBandEnd = ClosestPointOnLine(currentLineStartPoint, lockedAxisDir, snap.worldPoint);
        }
    } else {
        currentRubberBandEnd = snap.worldPoint;
    }
}

void LineTool::finalizeLine(const glm::vec3& endPoint) {
    if (glm::distance2(currentLineStartPoint, endPoint) > 1e-6f) {
        context.scene->AddUserLine(currentLineStartPoint, endPoint);
    }
    isPlacingSecondPoint = false;
    isPlacingFirstPoint = true;
    isAxisLocked = false;
    lineLengthInputBuf[0] = '\0';
}

void LineTool::RenderUI() {
    if (isPlacingSecondPoint) {
        ImGui::Separator();
        ImGui::Text("Length: %s", lineLengthInputBuf);
        ImGui::Separator();
    }
}

void LineTool::RenderPreview(Renderer& renderer, const SnapResult& snap) {
    if (isPlacingSecondPoint) {
        // We pass the start and end points directly to a new renderer function.
        renderer.UpdatePreviewLine(currentLineStartPoint, currentRubberBandEnd);
    } else {
        // Clear preview lines when not drawing
        renderer.UpdatePreviewLine(glm::vec3(0.0f), glm::vec3(0.0f), false);
    }
}

} // namespace Urbaxio::Tools 