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

    // --- NEW: Settings Accessors ---
    void SetPressureSensitivity(bool enabled);
    void SetBrushRadius(float radius); // Sets fixed/max radius
    void SetMinBrushRadius(float radius);
    void SetMaxBrushRadius(float radius);
    void SetBrushStrength(float strength);
    void SetAdditive(bool additive);
    // --- NEW: Scale Dependency Toggle ---
    void SetScaleDependent(bool enabled);
    // ------------------------------------
    // -------------------------------

    // ✅ Static initialization (called once at app startup)
    static void InitializePersistentResources();
    static void CleanupPersistentResources();

private:
    // --- MODIFIED: Added worldScale parameter ---
    bool drawStroke(const glm::vec3& worldPos, float worldScale);
    // --------------------------------------------
    void updateMesh();
    void checkForAsyncUpdate(bool forceSync = false);

    struct Impl;
    std::unique_ptr<Impl> impl_;

    // ✅ Persistent resources (live entire session)
    static std::unique_ptr<Engine::DynamicGpuHashGrid> s_sharedHashGrid;
    static std::atomic<bool> s_resourcesReady;
};

} // namespace Urbaxio::Shell

