#pragma once

#include "tools/ITool.h"
#include <memory>
#include <glm/glm.hpp>

namespace Urbaxio::Engine {
    class GpuVoxelManager;
    struct VoxelGrid;
    class SceneObject;
}

namespace Urbaxio::Shell {

class VrDrawTool : public Tools::ITool {
public:
    VrDrawTool();
    ~VrDrawTool() override;

    // ITool interface
    Tools::ToolType GetType() const override { return Tools::ToolType::SculptDraw; }
    const char* GetName() const override { return "VR Draw"; }
    void Activate(const Tools::ToolContext& context) override;
    void Deactivate() override;
    void OnUpdate(const SnapResult& snap, const glm::vec3& rayOrigin, const glm::vec3& rayDirection) override;
    void RenderUI() override;

    // VR-specific trigger events
    void OnTriggerPressed(bool isRightHand, const glm::vec3& pos);
    void OnTriggerHeld(bool isRightHand, const glm::vec3& pos);
    void OnTriggerReleased(bool isRightHand);

private:
    bool drawStroke(const glm::vec3& worldPos);
    void updateMeshRealtime();
    Engine::SceneObject* createChunk(const glm::ivec3& chunkCoord);

    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace Urbaxio::Shell

