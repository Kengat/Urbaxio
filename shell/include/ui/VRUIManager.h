#pragma once

#include "ui/VRPanel.h"
#include <map>
#include <string>

namespace Urbaxio {
    class Renderer;
    class TextRenderer;
}

namespace Urbaxio::UI {

class VRUIManager {
public:
    VRUIManager();

    VRPanel& AddPanel(const std::string& name, const std::string& displayName, const glm::vec2& size, const glm::mat4& offsetTransform, float cornerRadius, unsigned int grabIcon, unsigned int closeIcon, unsigned int minimizeIcon);
    VRPanel* GetPanel(const std::string& name);
    VRPanel* GetHoveredPanel();

    void Update(const Ray& worldRay, const glm::mat4& leftControllerTransform, const glm::mat4& rightControllerTransform, bool isClicked, bool isClickReleased, bool aButtonIsPressed, bool bButtonIsPressed);
    void Render(Renderer& renderer, TextRenderer& textRenderer, const glm::mat4& view, const glm::mat4& projection);
    bool HandleClick();
    bool IsInteracting() const;

private:
    std::map<std::string, VRPanel> panels_;
    std::string activeInteractionPanel_;
};

}

