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
    void OnUpdate(const SnapResult& snap, const glm::vec3& rayOrigin = {}, const glm::vec3& rayDirection = {}) override;
    void RenderPreview(Renderer& renderer, const SnapResult& snap) override;

    // --- Public methods to query drag state ---
    bool IsDragging() const;
    void GetDragRect(glm::vec2& outStart, glm::vec2& outCurrent) const;
    
    // NEW: Public methods for VR 3D drag box
    bool IsVrDragging() const;
    void GetVrDragBoxCorners(glm::vec3& outStart, glm::vec3& outEnd) const;


private:
    // Double-click detection state
    uint32_t lastClickTimestamp = 0;
    uint64_t lastClickedObjId = 0;
    size_t lastClickedTriangleIndex = 0;

    // Desktop drag selection state
    bool isMouseDown = false;
    bool isDragging = false;
    glm::vec2 dragStartCoords;
    glm::vec2 currentDragCoords;

    // VR interaction state
    bool isVrTriggerDown = false;
    bool isVrDragging = false;
    uint32_t vrTriggerDownTimestamp = 0;
    SnapResult vrClickSnapResult; // Stores the snap result at the moment of the click
    glm::vec3 vrDragStartPoint;
    glm::vec3 vrDragEndPoint;
};

} // namespace Urbaxio::Tools 