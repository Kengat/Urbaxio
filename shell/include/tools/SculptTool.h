#pragma once

#include "tools/ITool.h"

#include <openvdb/openvdb.h>
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

    // --- NEW: Setters for UI ---
    void SetBrushRadius(float radius);
    void SetBrushStrength(float strength);
    // ---------------------------

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
    // NEW: Bounding box for the 'before' state of a stroke
    openvdb::CoordBBox savedBeforeBBox_;
};

} // namespace Urbaxio::Tools
