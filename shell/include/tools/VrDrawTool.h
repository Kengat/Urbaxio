#pragma once

#include "ITool.h"
#include <memory>
#include <glm/glm.hpp>

// Forward declarations
namespace Urbaxio::Tools {
    struct ToolContext;
}

namespace Urbaxio::Shell {

class VrDrawTool : public Tools::ITool {
public:
    VrDrawTool();
    ~VrDrawTool() override;

    Tools::ToolType GetType() const override;
    const char* GetName() const override;
    void Activate(const Tools::ToolContext& context) override;
    void Deactivate() override;

    // VR-specific interface
    void OnTriggerPressed(bool isRightHand, const glm::vec3& pos);
    void OnTriggerHeld(bool isRightHand, const glm::vec3& pos);
    void OnTriggerReleased(bool isRightHand);

    // ITool interface
    void OnUpdate(const SnapResult& snap, const glm::vec3& rayOrigin = glm::vec3(0), const glm::vec3& rayDirection = glm::vec3(0)) override;
    void RenderUI() override;

private:
    bool drawStroke(const glm::vec3& worldPos);
    void updateMesh();
    void checkForAsyncUpdate(bool forceSync = false);

    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace Urbaxio::Shell

