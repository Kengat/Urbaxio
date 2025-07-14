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
    bool isPlacingFirstPoint = false;
    bool isPlacingSecondPoint = false;
    glm::vec3 currentLineStartPoint{0.0f};
    glm::vec3 currentRubberBandEnd{0.0f};

    // Axis Locking State
    bool isAxisLocked = false;
    SnapType lockedAxisType = SnapType::NONE;
    glm::vec3 lockedAxisDir{0.0f};

    // Input buffer for length
    char lineLengthInputBuf[64] = "";
    
    // The last snap result received by OnUpdate
    SnapResult lastSnapResult;

    void reset();
    void finalizeLine(const glm::vec3& endPoint);
};

} // namespace Urbaxio::Tools 