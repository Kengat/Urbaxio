#pragma once

#include "tools/ITool.h"
#include "snapping.h" // <-- FIX: Include header for SnapResult and SnapType

namespace Urbaxio::Tools {

class LineTool : public ITool {
public:
    ToolType GetType() const override { return ToolType::Line; }
    const char* GetName() const override { return "Line"; }

    void Activate(const ToolContext& context) override;
    void Deactivate() override;

    void OnLeftMouseDown(int mouseX, int mouseY, bool shift, bool ctrl, const glm::vec3& rayOrigin = {}, const glm::vec3& rayDirection = {}) override;
    void OnRightMouseDown() override;
    void OnKeyDown(SDL_Keycode key, bool shift, bool ctrl) override;
    void OnUpdate(const SnapResult& snap, const glm::vec3& rayOrigin = {}, const glm::vec3& rayDirection = {}) override;

    void RenderUI() override;
    void RenderPreview(Renderer& renderer, const SnapResult& snap) override;

    // --- Public Getters for external UI ---
    bool IsDrawing() const;
    float GetCurrentLineLength() const;
    void SetLengthInput(const std::string& input);

private:
    // --- NEW: State Machine ---
    enum class ToolState {
        IDLE,                               // Tool is active, but no line has been started
        AWAITING_SECOND_POINT_FREE,         // First point is placed, cursor moves freely
        AWAITING_SECOND_POINT_AXIS_LOCKED,  // First point is placed, movement is locked to a screen-aligned axis
        AWAITING_SECOND_POINT_INFERENCE_LOCKED // First point is placed, movement is locked to a user-defined direction
    };
    ToolState currentState = ToolState::IDLE;

    // --- State-related data ---
    glm::vec3 currentLineStartPoint{0.0f};
    glm::vec3 currentRubberBandEnd{0.0f};

    // --- Axis Locking State ---
    glm::vec3 lockedAxisDir{0.0f};
    SnapType lockedAxisType = SnapType::NONE;
    glm::vec3 inferenceAxisDir{0.0f}; // NEW: For user-defined direction lock

    // Input buffer for length
    char lengthInputBuf[64] = "";
    
    // The last snap result received by OnUpdate
    SnapResult lastSnapResult;

    // --- Private helper methods ---
    void reset();
    void finalizeLine(const glm::vec3& endPoint);
    bool tryToLockAxis(const glm::vec3& currentTarget);
    glm::vec3 calculateAxisLockedPoint(const SnapResult& snapResult, const glm::vec3& rayOrigin, const glm::vec3& rayDirection);
    glm::vec3 calculateInferenceLockedPoint(const SnapResult& snap, const glm::vec3& rayOrigin, const glm::vec3& rayDirection);
    bool isValidGeometricSnap(SnapType type); // <-- MOVED HERE
};

} // namespace Urbaxio::Tools 