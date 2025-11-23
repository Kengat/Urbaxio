#pragma once

#include "ui/VRPanel.h"
#include <map>
#include <string>
#include <glm/mat4x4.hpp>

namespace Urbaxio {
    class Renderer;
    class TextRenderer;
}

namespace Urbaxio::UI {

class VRUIManager {
public:
    VRUIManager();

    VRPanel& AddPanel(const std::string& name, const std::string& displayName, const glm::vec2& size, const glm::mat4& offsetTransform, float cornerRadius, unsigned int grabIcon, unsigned int pinIcon, unsigned int closeIcon, unsigned int minimizeIcon);
    VRPanel* GetPanel(const std::string& name);
    VRPanel* GetHoveredPanel();

    std::map<std::string, VRPanel>& GetPanels();
    const std::map<std::string, VRPanel>& GetPanels() const;

    void Update(const Ray& worldRay, const glm::mat4& leftControllerTransform, const glm::mat4& rightControllerTransform, bool triggerPressed, bool triggerReleased, bool triggerHeld, bool aButtonPressed, bool aButtonHeld, bool bButtonIsPressed, float leftStickY, bool isLeftTriggerPressed);
    void UpdateWithHeadTransform(const Ray& worldRay, const glm::mat4& leftControllerTransform, const glm::mat4& headTransform, const glm::mat4& rightControllerTransform, bool triggerPressed, bool triggerReleased, bool triggerHeld, bool aButtonPressed, bool aButtonHeld, bool bButtonIsPressed, float leftStickY, bool isLeftTriggerPressed);
    
    // --- NEW: Desktop methods ---
    void UpdateDesktop(const Ray& mouseRay, bool isLeftClick, bool isLeftHeld, bool isCtrlHeld, float scrollY);
    void RenderDesktop(Renderer& renderer, TextRenderer& textRenderer, const glm::mat4& orthoProjection);
    bool IsRayBlockedByPanelDesktop(const Ray& mouseRay) const;
    // ----------------------------

    void Render(Renderer& renderer, TextRenderer& textRenderer, const glm::mat4& view, const glm::mat4& projection);
    bool HandleClick();
    bool IsInteracting() const;
    bool IsRayBlockedByPanel(const Ray& worldRay, const glm::mat4& parentTransform) const;

private:
    std::map<std::string, VRPanel> panels_;
    std::string activeInteractionPanel_;
    glm::mat4 lastLeftControllerTransform_ = glm::mat4(1.0f);
    glm::mat4 lastHeadTransform_ = glm::mat4(1.0f);
    bool hasLastLeftTransform_ = false;
    bool hasLastHeadTransform_ = false;
};

}

