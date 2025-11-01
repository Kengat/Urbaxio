#pragma once

#include "tools/ITool.h"

namespace Urbaxio::Tools {

class PushPullTool : public ITool {
public:
    ToolType GetType() const override { return ToolType::PushPull; }
    const char* GetName() const override { return "Push/Pull"; }

    void Activate(const ToolContext& context) override;
    void Deactivate() override;

    // -- START OF MODIFICATION --
    void OnLeftMouseDown(int mouseX, int mouseY, bool shift, bool ctrl, const glm::vec3& rayOrigin, const glm::vec3& rayDirection) override;
    // -- END OF MODIFICATION --
    void OnRightMouseDown() override;
    void OnKeyDown(SDL_Keycode key, bool shift, bool ctrl) override;
    // -- START OF MODIFICATION --

    void OnUpdate(const SnapResult& snap, const glm::vec3& rayOrigin, const glm::vec3& rayDirection) override;

    void OnMouseMove(int mouseX, int mouseY) override;

    // -- END OF MODIFICATION --

    void RenderUI() override;
    void RenderPreview(Renderer& renderer, const SnapResult& snap) override;

    // State query method for snapping control
    bool IsPushPullActive() const { return isPushPullActive; }

    // Set length input from VR numpad
    void SetLengthInput(const std::string& input);

    // -- START OF MODIFICATION --

    // This is now the primary hover update function, used by both VR and 2D.

    void updateHover(const glm::vec3& rayOrigin, const glm::vec3& rayDirection);

    // -- END OF MODIFICATION --

private:
    bool isPushPullActive = false;
    
    // State for the active operation
    uint64_t pushPull_objId = 0;
    std::vector<size_t> pushPull_faceIndices;
    glm::vec3 pushPull_faceNormal{0.0f};
    glm::vec3 pushPull_startPoint{0.0f}; // Point on face plane where drag started
    float pushPullCurrentLength = 0.0f;
    int pushPull_startMouseX = 0;
    int pushPull_startMouseY = 0;
    
    // Input buffer for length
    char lengthInputBuf[64] = "";
    
    void reset();
    void finalizePushPull(bool ctrl);

};

} // namespace Urbaxio::Tools 