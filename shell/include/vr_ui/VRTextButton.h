// shell/include/vr_ui/VRTextButton.h
#pragma once

#include "vr_ui/VRWidget.h"
#include <string>

namespace Urbaxio::VRUI {

class VRTextButton : public VRWidget {
public:
    VRTextButton(const std::string& label, const glm::mat4& localTransform, float radius = 0.028f);

    void Update(const VRInputState& input, const VRUIStyle& style) override;
    void Render(Renderer& renderer, TextRenderer& textRenderer, const glm::mat4& view, const glm::mat4& projection, const VRUIStyle& style) override;
    float CheckHit(const VRInputState& input) override;

    std::function<void()> OnClick;

private:
    std::string label_;
    float radius_;
    float hoverAlpha_ = 0.7f;
};

} // namespace Urbaxio::VRUI

