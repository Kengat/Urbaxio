#define GLM_ENABLE_EXPERIMENTAL
#include "ui/VRUIManager.h"
#include "renderer.h"
#include "TextRenderer.h"
#include <limits>

namespace Urbaxio::UI {

VRUIManager::VRUIManager() {}

VRPanel& VRUIManager::AddPanel(const std::string& name, const std::string& displayName, const glm::vec2& size, const glm::mat4& offsetTransform, float cornerRadius, unsigned int grabIcon, unsigned int closeIcon, unsigned int minimizeIcon) {
    auto [it, success] = panels_.try_emplace(name, name, displayName, size, offsetTransform, cornerRadius, grabIcon, closeIcon, minimizeIcon);
    return it->second;
}

VRPanel* VRUIManager::GetPanel(const std::string& name) {
    auto it = panels_.find(name);
    if (it != panels_.end()) {
        return &it->second;
    }
    return nullptr;
}

VRPanel* VRUIManager::GetHoveredPanel() {
    if (!activeInteractionPanel_.empty()) {
        return GetPanel(activeInteractionPanel_);
    }
    return nullptr;
}

void VRUIManager::Update(const Ray& worldRay, const glm::mat4& leftControllerTransform, const glm::mat4& rightControllerTransform, bool isClicked, bool isClickReleased, bool aButtonIsPressed, bool bButtonIsPressed) {
    const glm::mat4& parentTransform = leftControllerTransform;
    const glm::mat4& interactionTransform = rightControllerTransform;

    std::string newActivePanel;
    float closestHitDist = std::numeric_limits<float>::max();

    for (auto& [name, panel] : panels_) {
        if (panel.IsVisible() && panel.alpha > 0.01f) {
            HitResult hit = panel.CheckIntersection(worldRay, parentTransform);
            if (hit.didHit && hit.distance < closestHitDist) {
                closestHitDist = hit.distance;
                newActivePanel = name;
            }
        }
    }
    activeInteractionPanel_ = newActivePanel;
    
    for (auto& [name, panel] : panels_) {
        if (panel.IsVisible()) {
            // Update all panels, but only the active one gets click events
            bool panelIsClicked = (name == activeInteractionPanel_) && isClicked;
            panel.Update(worldRay, parentTransform, interactionTransform, panelIsClicked, isClickReleased, aButtonIsPressed, bButtonIsPressed);
        }
    }
}

void VRUIManager::Render(Urbaxio::Renderer& renderer, Urbaxio::TextRenderer& textRenderer, const glm::mat4& view, const glm::mat4& projection) {
    for (auto& [name, panel] : panels_) {
        panel.Render(renderer, textRenderer, view, projection);
    }
}

bool VRUIManager::HandleClick() {
    if (!activeInteractionPanel_.empty()) {
        auto it = panels_.find(activeInteractionPanel_);
        if (it != panels_.end()) {
            return it->second.HandleClick();
        }
    }
    return false;
}

bool VRUIManager::IsInteracting() const {
    for (const auto& [name, panel] : panels_) {
        if (panel.isGrabbing || panel.IsResizing() || panel.IsChangingProportions()) {
            return true;
        }
    }
    return false;
}

}

