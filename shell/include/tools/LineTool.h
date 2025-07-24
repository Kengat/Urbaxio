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

    void OnLeftMouseDown(int mouseX, int mouseY, bool shift, bool ctrl) override;
    void OnRightMouseDown() override;
    void OnKeyDown(SDL_Keycode key, bool shift, bool ctrl) override;
    void OnUpdate(const SnapResult& snap) override;

    void RenderUI() override;
    void RenderPreview(Renderer& renderer, const SnapResult& snap) override;

private:
    // --- NEW: State Machine ---
    enum class ToolState {
        IDLE,                           // Tool is active, but no line has been started
        AWAITING_SECOND_POINT_FREE,     // First point is placed, cursor moves freely
        AWAITING_SECOND_POINT_AXIS_LOCKED // First point is placed, movement is locked to an axis
    };
    ToolState currentState = ToolState::IDLE;

    // --- State-related data ---
    glm::vec3 currentLineStartPoint{0.0f};
    glm::vec3 currentRubberBandEnd{0.0f};

    // --- Axis Locking State ---
    glm::vec3 lockedAxisDir{0.0f};
    SnapType lockedAxisType = SnapType::NONE;

    // Input buffer for length
    char lineLengthInputBuf[64] = "";
    
    // The last snap result received by OnUpdate
    SnapResult lastSnapResult;

    // --- Private helper methods ---
    void reset();
    void finalizeLine(const glm::vec3& endPoint);
    bool tryToLockAxis(const glm::vec3& currentTarget);
    glm::vec3 calculateAxisLockedPoint(const SnapResult& snapResult);
    bool isValidGeometricSnap(SnapType type);
};

} // namespace Urbaxio::Tools 