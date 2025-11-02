// shell/include/vr_ui/VRBillboardButton.h
#pragma once
#include "vr_ui/VRWidget.h"

namespace Urbaxio::VRUI {

class VRBillboardButton : public VRWidget {
public:
    VRBillboardButton(const glm::mat4& localTransform, float radius = 0.015f);

    void Update(const VRInputState& input, const VRUIStyle& style) override;
    void Render(Renderer& renderer, TextRenderer& textRenderer, const glm::mat4& view, const glm::mat4& projection, const VRUIStyle& style) override;
    float CheckHit(const VRInputState& input) override;
    
    std::function<void()> OnClick;
    std::function<void(bool)> OnPressStateChanged;

    // Style properties
    glm::vec3 baseColor = {1.0f, 1.0f, 1.0f};

private:
    float radius_;
    float hoverAberration_ = 0.05f;
    bool wasPressed_ = false;
};

}

