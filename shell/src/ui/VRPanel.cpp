#define GLM_ENABLE_EXPERIMENTAL
#include "ui/VRPanel.h"
#include "renderer.h"
#include "TextRenderer.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/intersect.hpp>
#include <limits>

namespace Urbaxio::UI {

VRPanel::VRPanel(const std::string& name, const glm::vec2& size, const glm::mat4& offsetTransform, float cornerRadius)
    : name_(name), size_(size), offsetTransform_(offsetTransform), cornerRadius_(cornerRadius), hoveredWidget_(nullptr), isGrabbing(false), isHoveringGrabHandle(false) {}

void VRPanel::AddWidget(std::unique_ptr<IVRWidget> widget) {
    widgets_.push_back(std::move(widget));
}

void VRPanel::Update(const Ray& worldRay, const glm::mat4& parentTransform, bool isClicked) {
    if (!isGrabbing) {
        transform = parentTransform * offsetTransform_;
    }

    glm::mat4 invTransform = glm::inverse(transform);
    Ray localRay;
    localRay.origin = invTransform * glm::vec4(worldRay.origin, 1.0f);
    localRay.direction = glm::normalize(glm::vec3(invTransform * glm::vec4(worldRay.direction, 0.0f)));

    IVRWidget* newHoveredWidget = nullptr;
    float closestHitDist = std::numeric_limits<float>::max();

    for (auto& widget : widgets_) {
        HitResult hit = widget->CheckIntersection(localRay);
        if (hit.didHit && hit.distance < closestHitDist) {
            closestHitDist = hit.distance;
            newHoveredWidget = widget.get();
        }
    }
    
    if (hoveredWidget_ != newHoveredWidget) {
        if (hoveredWidget_) {
            hoveredWidget_->SetHover(false);
        }
        hoveredWidget_ = newHoveredWidget;
        if (hoveredWidget_) {
            hoveredWidget_->SetHover(true);
        }
    }
    
    // --- MODIFIED: Update ALL widgets every frame so they can handle animations ---
    for (auto& widget : widgets_) {
        widget->Update(localRay, false); // isClicked is handled by HandleClick()
    }
}

void VRPanel::Render(Urbaxio::Renderer& renderer, Urbaxio::TextRenderer& textRenderer, const glm::mat4& view, const glm::mat4& projection) {
    if (!isVisible_ || alpha < 0.01f) return;

    // Render panel background
    glm::mat4 backgroundModel = transform * glm::scale(glm::mat4(1.0f), glm::vec3(size_.x, size_.y, 1.0f));
    renderer.RenderVRPanel(view, projection, backgroundModel, glm::vec3(0.43f, 0.65f, 0.82f), cornerRadius_, 0.25f * alpha);
    
    textRenderer.SetPanelModelMatrix(transform);

    // Render all widgets on this panel
    for (auto& widget : widgets_) {
        widget->Render(renderer, textRenderer, transform, view, projection, alpha);
    }

    textRenderer.Render(view, projection);
}

HitResult VRPanel::CheckIntersection(const Ray& worldRay, const glm::mat4& parentTransform) {
    HitResult result;
    if (!isVisible_) return result;
    
    glm::mat4 finalTransform = parentTransform * offsetTransform_;
    
    glm::vec3 panelOrigin = glm::vec3(finalTransform[3]);
    glm::vec3 panelNormal = -glm::normalize(glm::vec3(finalTransform[2]));
    
    float t;
    if (glm::intersectRayPlane(worldRay.origin, worldRay.direction, panelOrigin, panelNormal, t) && t > 0) {
        glm::vec3 hitPoint = worldRay.origin + worldRay.direction * t;
        
        glm::vec3 localHitPoint = glm::inverse(finalTransform) * glm::vec4(hitPoint, 1.0f);
        if (glm::abs(localHitPoint.x) <= size_.x * 0.5f && glm::abs(localHitPoint.y) <= size_.y * 0.5f) {
            result.didHit = true;
            result.distance = t;
        }
    }
    return result;
}

void VRPanel::SetVisible(bool visible) {
    isVisible_ = visible;
}

bool VRPanel::IsVisible() const {
    return isVisible_;
}

const std::string& VRPanel::GetName() const {
    return name_;
}

bool VRPanel::HandleClick() {
    if (hoveredWidget_) { 
        hoveredWidget_->HandleClick(); 
        return true; 
    }
    return false;
}

}

