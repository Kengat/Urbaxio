#pragma once

#include "tools/ITool.h"
#include <glm/mat4x4.hpp>

namespace Urbaxio::Tools {

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
    
    // Public getters for the main loop to query state for rendering
    bool IsMoving() const;
    uint64_t GetMovingObjectId() const;
    glm::mat4 GetPreviewTransform() const;

private:
    enum class State {
        IDLE,                   // Waiting for user to select an object and click a base point
        AWAITING_DESTINATION    // Base point is set, dragging object to destination
    };

    State currentState = State::IDLE;
    uint64_t movingObjectId = 0;
    glm::vec3 basePoint{0.0f};
    glm::vec3 currentTranslation{0.0f};

    void reset();
    void finalizeMove();
};

} // namespace Urbaxio::Tools