#include "ui/VRUIManager.h"
#include "renderer.h"
#include "TextRenderer.h"
#include <limits>

namespace Urbaxio::UI {

VRUIManager::VRUIManager() {}

VRPanel& VRUIManager::AddPanel(const std::string& name, const glm::vec2& size, const glm::mat4& offsetTransform, float cornerRadius) {
    auto [it, success] = panels_.try_emplace(name, name, size, offsetTransform, cornerRadius);
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

void VRUIManager::Update(const Ray& worldRay, const glm::mat4& leftControllerTransform, const glm::mat4& rightControllerTransform, bool isClicked) {
    const glm::mat4& parentTransform = leftControllerTransform;

    std::string newActivePanel;
    float closestHitDist = std::numeric_limits<float>::max();

    for (auto& [name, panel] : panels_) {
        // Only check for interaction if the panel is actually visible
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
        // The alpha fading logic is now handled exclusively in main.cpp based on trigger state.
        // We just need to update the panel's state.

        if (panel.IsVisible() && name == activeInteractionPanel_) {
            panel.Update(worldRay, parentTransform, isClicked);
        } else {
            // Even if not active for interaction, we need to update its position and clear its hover state.
            // Pass 'false' for isClicked to prevent actions.
            panel.Update(worldRay, parentTransform, false); 
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

}

