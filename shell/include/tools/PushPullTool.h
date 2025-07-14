#pragma once

#include "tools/ITool.h"

namespace Urbaxio::Tools {

class PushPullTool : public ITool {
public:
    ToolType GetType() const override { return ToolType::PushPull; }
    const char* GetName() const override { return "Push/Pull"; }

    void Activate(const ToolContext& context) override;
    void Deactivate() override;

    void OnLeftMouseDown(int mouseX, int mouseY, bool shift, bool ctrl) override;
    void OnRightMouseDown() override;
    void OnKeyDown(SDL_Keycode key, bool shift, bool ctrl) override;
    void OnUpdate(const SnapResult& snap) override;

    void RenderUI() override;
    void RenderPreview(Renderer& renderer, const SnapResult& snap) override;

    // State query method for snapping control
    bool IsPushPullActive() const { return isPushPullActive; }
private:
    bool isPushPullActive = false;
    
    // State for the active operation
    uint64_t pushPull_objId = 0;
    std::vector<size_t> pushPull_faceIndices;
    glm::vec3 pushPull_faceNormal{0.0f};
    glm::vec3 pushPull_startPoint{0.0f}; // Point on face plane where drag started
    float pushPullCurrentDistance = 0.0f;
    int pushPull_startMouseX = 0;
    int pushPull_startMouseY = 0;
    
    // Input buffer for distance
    char distanceInputBuf[64] = "";
    
    void reset();
    void finalizePushPull(bool ctrl);
    void updateHover(int mouseX, int mouseY);
};

} // namespace Urbaxio::Tools 