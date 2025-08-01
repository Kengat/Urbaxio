#pragma once

#include "tools/ITool.h"
#include <glm/mat4x4.hpp>

namespace Urbaxio::Tools {

// FIX: Moved the enum class outside of the MoveTool class
// This resolves the MSVC compiler issue with qualified names in switch statements.
enum class MoveToolState {
    IDLE,
    AWAITING_DESTINATION_FREE,
    AWAITING_DESTINATION_AXIS_LOCKED,
    AWAITING_DESTINATION_INFERENCE_LOCKED
};

class MoveTool : public ITool {
public:
    ToolType GetType() const override { return ToolType::Move; }
    const char* GetName() const override { return "Move"; }

    void Activate(const ToolContext& context) override;
    void Deactivate() override;

    void OnLeftMouseDown(int mouseX, int mouseY, bool shift, bool ctrl) override;
    void OnRightMouseDown() override;
    void OnUpdate(const SnapResult& snap) override;
    void OnKeyDown(SDL_Keycode key, bool shift, bool ctrl) override;
    void RenderUI() override;
    
    // Public getters for the main loop to query state for rendering
    bool IsMoving() const;
    uint64_t GetMovingObjectId() const;
    glm::mat4 GetPreviewTransform() const;

private:
    MoveToolState currentState = MoveToolState::IDLE;
    uint64_t movingObjectId = 0;
    glm::vec3 basePoint{0.0f};
    glm::vec3 currentTranslation{0.0f};
    
    // --- NEW: Axis Locking State ---
    glm::vec3 lockedAxisDir{0.0f};
    SnapType lockedAxisType = SnapType::NONE;
    glm::vec3 inferenceAxisDir{0.0f};

    // Input buffer for length
    char lengthInputBuf[64] = "";

    void reset();
    void finalizeMove();
    bool tryToLockAxis(const glm::vec3& currentTarget);
    glm::vec3 calculateAxisLockedPoint(const SnapResult& snap);
    glm::vec3 calculateInferenceLockedPoint(const SnapResult& snap);
};

} // namespace Urbaxio::Tools