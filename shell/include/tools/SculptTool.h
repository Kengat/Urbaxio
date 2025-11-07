#pragma once

#include "tools/ITool.h"

#include <glm/glm.hpp>

namespace Urbaxio::Tools {

class SculptTool : public ITool {
public:
    ToolType GetType() const override { return ToolType::Sculpt; }
    const char* GetName() const override { return "Sculpt"; }

    void Activate(const ToolContext& context) override;
    void Deactivate() override;
    
    // We now handle clicks here
    void OnLeftMouseDown(int mouseX, int mouseY, bool shift, bool ctrl, const glm::vec3& rayOrigin, const glm::vec3& rayDirection) override;
    void OnLeftMouseUp(int mouseX, int mouseY, bool shift, bool ctrl) override;
    
    void OnUpdate(const SnapResult& snap, const glm::vec3& rayOrigin = {}, const glm::vec3& rayDirection = {}) override;
    
    void RenderUI() override;
    void RenderPreview(Renderer& renderer, const SnapResult& snap) override;

private:
    bool applyBrush(const glm::vec3& brushWorldPos);
    void updateBrushCursor(const glm::vec3& position, bool visible);
    void reset();

    // Brush settings
    float brushRadius_ = 1.0f;
    float brushStrength_ = 0.5f;

    // State
    bool isSculpting_ = false;
    uint64_t sculptedObjectId_ = 0; // NEW: Keep track of the object being sculpted
    uint64_t brushCursorObjId_ = 0;
    std::vector<float> gridDataBeforeStroke_;
    glm::vec3 lastBrushApplyPos_{0.0f};
    std::vector<float> workingGridData_;
};

} // namespace Urbaxio::Tools
