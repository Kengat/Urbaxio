#pragma once

#include "tools/ITool.h"
#include <memory>

namespace Urbaxio::Engine {
    class GpuVoxelManager;
    struct VoxelGrid;
}

namespace Urbaxio::Shell {

class GpuSculptTool : public Tools::ITool {
public:
    GpuSculptTool();
    ~GpuSculptTool() override;

    // ITool interface
    Tools::ToolType GetType() const override { return Tools::ToolType::Sculpt; }
    const char* GetName() const override { return "GPU Sculpt"; }
    void Activate(const Tools::ToolContext& context) override;
    void Deactivate() override;
    void OnLeftMouseDown(int mouseX, int mouseY, bool shift, bool ctrl, const glm::vec3& rayOrigin = {}, const glm::vec3& rayDirection = {}) override;
    void OnLeftMouseUp(int mouseX, int mouseY, bool shift, bool ctrl) override;
    void OnMouseMove(int mouseX, int mouseY) override;
    void OnUpdate(const SnapResult& snap, const glm::vec3& rayOrigin = {}, const glm::vec3& rayDirection = {}) override;
    void RenderUI() override;

private:
    bool applyBrushGPU(const glm::vec3& brushWorldPos);
    bool applyBrushBatchGPU(); // Apply batched strokes

    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace Urbaxio::Shell

