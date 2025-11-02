// shell/include/vr_ui/VRTextDisplay.h
#pragma once
#include "vr_ui/VRWidget.h"
#include <string>

namespace Urbaxio::VRUI {

class VRTextDisplay : public VRWidget {
public:
    VRTextDisplay(std::string* textSource, const glm::mat4& localTransform);
    
    void Update(const VRInputState& input, const VRUIStyle& style) override;
    void Render(Renderer& renderer, TextRenderer& textRenderer, const glm::mat4& view, const glm::mat4& projection, const VRUIStyle& style) override;
    float CheckHit(const VRInputState& input) override;

private:
    std::string* textSource_; // Pointer to the string to display
};

} // namespace Urbaxio::VRUI

