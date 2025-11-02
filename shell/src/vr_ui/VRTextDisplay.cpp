// shell/src/vr_ui/VRTextDisplay.cpp
#include "vr_ui/VRTextDisplay.h"
#include "renderer.h"
#include "TextRenderer.h"

namespace Urbaxio::VRUI {

VRTextDisplay::VRTextDisplay(std::string* textSource, const glm::mat4& localTransform)
    : VRWidget(localTransform), textSource_(textSource) {}

void VRTextDisplay::Update(const VRInputState& input, const VRUIStyle& style) {
    isHovered_ = false;
}

void VRTextDisplay::Render(Renderer& renderer, TextRenderer& textRenderer, const glm::mat4& view, const glm::mat4& projection, const VRUIStyle& style) {
    if (!textSource_) return;
    
    renderer.RenderVRPanel(view, projection, finalTransform_, style.panelColor, style.panelCornerRadius * 0.5f, style.panelAlpha);

    textRenderer.SetPanelModelMatrix(finalTransform_);
    float panelScale = glm::length(glm::vec3(parentTransform_[0]));
    textRenderer.AddTextOnPanel(*textSource_, glm::mat4(1.0f), style.textColor, 0.03f * panelScale);
}

float VRTextDisplay::CheckHit(const VRInputState& input) {
    return -1.0f;
}

} // namespace Urbaxio::VRUI

