#define GLM_ENABLE_EXPERIMENTAL
#include "ui/VRUIManager.h"
#include "renderer.h"
#include "TextRenderer.h"
#include <limits>
#include <vector>
#include <algorithm>
#include <set>

namespace Urbaxio::UI {

VRUIManager::VRUIManager() {}

std::vector<VRPanel*> GetSortedPanels(std::map<std::string, VRPanel>& panels) {
    std::vector<VRPanel*> sorted;
    sorted.reserve(panels.size());
    std::set<VRPanel*> processed;

    for (auto& [name, panel] : panels) {
        if (!panel.GetParent()) {
            sorted.push_back(&panel);
            processed.insert(&panel);
        }
    }

    bool added = true;
    while (added) {
        added = false;
        for (auto& [name, panel] : panels) {
            if (processed.count(&panel)) continue;
            if (processed.count(panel.GetParent())) {
                sorted.push_back(&panel);
                processed.insert(&panel);
                added = true;
            }
        }
    }

    for (auto& [name, panel] : panels) {
        if (!processed.count(&panel)) {
            sorted.push_back(&panel);
        }
    }

    return sorted;
}

VRPanel& VRUIManager::AddPanel(const std::string& name, const std::string& displayName, const glm::vec2& size, const glm::mat4& offsetTransform, float cornerRadius, unsigned int grabIcon, unsigned int pinIcon, unsigned int closeIcon, unsigned int minimizeIcon) {
    auto [it, success] = panels_.try_emplace(name, name, displayName, size, offsetTransform, cornerRadius, grabIcon, pinIcon, closeIcon, minimizeIcon);
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

std::map<std::string, VRPanel>& VRUIManager::GetPanels() {
    return panels_;
}

void VRUIManager::Update(const Ray& worldRay, const glm::mat4& leftControllerTransform, const glm::mat4& rightControllerTransform, bool triggerPressed, bool triggerReleased, bool triggerHeld, bool aButtonPressed, bool aButtonHeld, bool bButtonIsPressed, float leftStickY, bool isLeftTriggerPressed) {
    const glm::mat4& interactionTransform = rightControllerTransform;

    lastLeftControllerTransform_ = leftControllerTransform;
    hasLastLeftTransform_ = true;
    lastHeadTransform_ = leftControllerTransform;
    hasLastHeadTransform_ = false;

    std::string newActivePanel;
    float closestHitDist = std::numeric_limits<float>::max();

    std::vector<VRPanel*> sortedPanels = GetSortedPanels(panels_);

    for (VRPanel* panelPtr : sortedPanels) {
        VRPanel& panel = *panelPtr;
        // --- FIX: Only interact if panel is visible AND opaque enough (>20%) ---
        if (panel.IsVisible() && panel.alpha > 0.2f) {
            glm::mat4 parentTransform = panel.GetParent() ? panel.GetParent()->transform : leftControllerTransform;
            HitResult hit = panel.CheckIntersection(worldRay, parentTransform);
            if (hit.didHit && hit.distance < closestHitDist) {
                closestHitDist = hit.distance;
                newActivePanel = panel.GetName();
            }
        }
    }
    activeInteractionPanel_ = newActivePanel;
    
    for (VRPanel* panelPtr : sortedPanels) {
        VRPanel& panel = *panelPtr;
        glm::mat4 parentTransform = panel.GetParent() ? panel.GetParent()->transform : leftControllerTransform;
        float stickForPanel = (panel.GetName() == activeInteractionPanel_) ? leftStickY : 0.0f;
        panel.Update(worldRay, parentTransform, interactionTransform, triggerPressed, triggerReleased, triggerHeld, aButtonPressed, aButtonHeld, bButtonIsPressed, stickForPanel, isLeftTriggerPressed);
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

const std::map<std::string, VRPanel>& VRUIManager::GetPanels() const {
    return panels_;
}

bool VRUIManager::IsRayBlockedByPanel(const Ray& worldRay, const glm::mat4& parentTransform) const {
    for (const auto& [name, panel] : panels_) {
        // --- FIX: Allow ray to pass through if panel is mostly transparent (< 20%) ---
        if (panel.IsVisible() && panel.alpha > 0.2f) {
            glm::mat4 effectiveParent;
            if (panel.GetParent()) {
                effectiveParent = panel.GetParent()->transform;
            } else {
                const glm::mat4* transformToUse = nullptr;
                if (panel.IsPinned()) {
                    if (hasLastHeadTransform_) {
                        transformToUse = &lastHeadTransform_;
                    }
                } else {
                    if (hasLastLeftTransform_) {
                        transformToUse = &lastLeftControllerTransform_;
                    }
                }
                effectiveParent = transformToUse ? *transformToUse : parentTransform;
            }

            HitResult hit = panel.CheckIntersection(worldRay, effectiveParent);
            if (hit.didHit) {
                return true;
            }
        }
    }
    return false;
}

void VRUIManager::UpdateWithHeadTransform(const Ray& worldRay, const glm::mat4& leftControllerTransform, const glm::mat4& headTransform, const glm::mat4& rightControllerTransform, bool triggerPressed, bool triggerReleased, bool triggerHeld, bool aButtonPressed, bool aButtonHeld, bool bButtonIsPressed, float leftStickY, bool isLeftTriggerPressed) {
    const glm::mat4& interactionTransform = rightControllerTransform;

    lastLeftControllerTransform_ = leftControllerTransform;
    hasLastLeftTransform_ = true;
    lastHeadTransform_ = headTransform;
    hasLastHeadTransform_ = true;

    std::string newActivePanel;
    float closestHitDist = std::numeric_limits<float>::max();

    std::vector<VRPanel*> sortedPanels = GetSortedPanels(panels_);

    for (VRPanel* panelPtr : sortedPanels) {
        VRPanel& panel = *panelPtr;
        // --- FIX: Only interact if panel is visible AND opaque enough (>20%) ---
        if (panel.IsVisible() && panel.alpha > 0.2f) {
            glm::mat4 parentTransform;
            if (panel.GetParent()) {
                parentTransform = panel.GetParent()->transform;
            } else {
                parentTransform = panel.IsPinned() ? headTransform : leftControllerTransform;
            }

            HitResult hit = panel.CheckIntersection(worldRay, parentTransform);
            if (hit.didHit && hit.distance < closestHitDist) {
                closestHitDist = hit.distance;
                newActivePanel = panel.GetName();
            }
        }
    }
    activeInteractionPanel_ = newActivePanel;
    
    for (VRPanel* panelPtr : sortedPanels) {
        VRPanel& panel = *panelPtr;

        glm::mat4 parentTransform;
        if (panel.GetParent()) {
            parentTransform = panel.GetParent()->transform;
        } else {
            parentTransform = panel.IsPinned() ? headTransform : leftControllerTransform;
        }

        float stickForPanel = (panel.GetName() == activeInteractionPanel_) ? leftStickY : 0.0f;
        panel.Update(worldRay, parentTransform, interactionTransform, triggerPressed, triggerReleased, triggerHeld, aButtonPressed, aButtonHeld, bButtonIsPressed, stickForPanel, isLeftTriggerPressed);
        
        if (panel.WasPinButtonClicked()) {
            bool newPinnedState = !panel.IsPinned();
            const glm::mat4& currentParent = panel.IsPinned() ? headTransform : leftControllerTransform;
            const glm::mat4& newParent = newPinnedState ? headTransform : leftControllerTransform;
            panel.SetPinned(newPinnedState, currentParent, newParent);
        }
    }
}

// --- NEW: Desktop Implementation ---
void VRUIManager::UpdateDesktop(const Ray& mouseRay, bool isLeftClick, bool isLeftHeld, bool isCtrlHeld, float scrollY) {
    // Simple iteration, order doesn't matter much for update logic as panels track grab state independently
    // Reverse iterator might be better for click occlusion, but CheckIntersection handles depth in Panels
    
    for (auto& [name, panel] : panels_) {
        // In desktop mode, we assume panels are top-level or parented to each other, not controllers
        panel.UpdateDesktop(mouseRay, isLeftClick, isLeftHeld, isCtrlHeld, scrollY);
    }
}

void VRUIManager::RenderDesktop(Renderer& renderer, TextRenderer& textRenderer, const glm::mat4& orthoProjection) {
    glm::mat4 identityView(1.0f);
    
    // Sort by Z position? In 2D we might want manual Z-ordering (e.g. active panel on top).
    // For now, simple iteration. Since we clear depth buffer before UI pass, 
    // painters algorithm (draw order) determines visibility.
    
    // We use the standard Render method but with Identity View and Ortho Projection.
    for (auto& [name, panel] : panels_) {
        panel.Render(renderer, textRenderer, identityView, orthoProjection);
    }
}

bool VRUIManager::IsRayBlockedByPanelDesktop(const Ray& mouseRay) const {
    glm::mat4 identity(1.0f);
    for (const auto& [name, panel] : panels_) {
        if (panel.IsVisible() && panel.alpha > 0.2f) {
            // In desktop mode, parent is effectively identity
            HitResult hit = panel.CheckIntersection(mouseRay, identity);
            if (hit.didHit) {
                return true;
            }
        }
    }
    return false;
}
// -----------------------------------

}

