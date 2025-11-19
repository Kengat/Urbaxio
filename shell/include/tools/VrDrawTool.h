#pragma once

#include "ITool.h"
#include <memory>
#include <glm/glm.hpp>
#include <atomic>

// Forward declarations
namespace Urbaxio::Engine {
    class DynamicGpuHashGrid; // Forward declaration
}

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
    void RenderPreview(Renderer& renderer, const SnapResult& snap) override;

    // ✅ Static initialization (called once at app startup)
    static void InitializePersistentResources();
    static void CleanupPersistentResources();

private:
    bool drawStroke(const glm::vec3& worldPos);
    void updateMesh();
    void checkForAsyncUpdate(bool forceSync = false);

    struct Impl;
    std::unique_ptr<Impl> impl_;

    // ✅ Persistent resources (live entire session)
    static std::unique_ptr<Engine::DynamicGpuHashGrid> s_sharedHashGrid;
    static std::atomic<bool> s_resourcesReady;
};

} // namespace Urbaxio::Shell

