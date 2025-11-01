#pragma once

#include "tools/ITool.h"
#include <glm/glm.hpp> // For glm::vec2
#include <cstdint>
#include <cstddef> // for size_t

namespace Urbaxio::Tools {

class SelectTool : public ITool {
public:
    ToolType GetType() const override { return ToolType::Select; }
    const char* GetName() const override { return "Select"; }

    void Activate(const ToolContext& context) override;
    void Deactivate() override;

    void OnLeftMouseDown(int mouseX, int mouseY, bool shift, bool ctrl, const glm::vec3& rayOrigin = {}, const glm::vec3& rayDirection = {}) override;
    void OnLeftMouseUp(int mouseX, int mouseY, bool shift, bool ctrl) override;
    void OnMouseMove(int mouseX, int mouseY) override;

    // --- Public methods to query drag state ---
    bool IsDragging() const;
    void GetDragRect(glm::vec2& outStart, glm::vec2& outCurrent) const;


private:
    // Double-click detection state
    uint32_t lastClickTimestamp = 0;
    uint64_t lastClickedObjId = 0;
    size_t lastClickedTriangleIndex = 0;

    // --- NEW: Drag selection state ---
    bool isMouseDown = false; // Is the left mouse button currently held down?
    bool isDragging = false;  // Has the mouse moved past the threshold while down?
    glm::vec2 dragStartCoords;
    glm::vec2 currentDragCoords;
};

} // namespace Urbaxio::Tools 