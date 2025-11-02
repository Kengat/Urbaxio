#define GLM_ENABLE_EXPERIMENTAL
#include "ui/VRPanel.h"
#include "renderer.h"
#include "TextRenderer.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/intersect.hpp>
#include <limits>

namespace Urbaxio::UI {

VRPanel::VRPanel(const std::string& name, const glm::vec2& size, const glm::mat4& offsetTransform)
    : name_(name), size_(size), offsetTransform_(offsetTransform), hoveredWidget_(nullptr), isGrabbing(false), isHoveringGrabHandle(false) {}

void VRPanel::AddWidget(std::unique_ptr<IVRWidget> widget) {
    widgets_.push_back(std::move(widget));
}

void VRPanel::Update(const Ray& worldRay, const glm::mat4& parentTransform, bool isClicked) {
    if (!isGrabbing) {
        transform = parentTransform * offsetTransform_;
    }

    // --- GRAB HANDLE HOVER CHECK (copied from main.cpp) ---
    glm::vec3 grabHandleCenter = GetGrabHandleCenter(); // Use helper

    isHoveringGrabHandle = false;
    float dist;
    float grabHandleRadius = 0.012f * glm::length(glm::vec3(transform[0]));
    if (glm::intersectRaySphere(worldRay.origin, worldRay.direction, grabHandleCenter, grabHandleRadius * grabHandleRadius, dist)) {
        isHoveringGrabHandle = true;
    }

    const float FADE_SPEED = 0.15f;
    float targetAlpha = isHoveringGrabHandle ? 1.0f : 0.0f;
    grabHandleHoverAlpha_ += (targetAlpha - grabHandleHoverAlpha_) * FADE_SPEED;

    if (isHoveringGrabHandle) {
        if(hoveredWidget_) hoveredWidget_->SetHover(false);
        hoveredWidget_ = nullptr;
        return;
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
    
    // Grab Handle
    float panelLocalScale = glm::length(glm::vec3(transform[0]));
    float grabHandleRadius = 0.012f * panelLocalScale;
    glm::vec3 grabHandleCenter = GetGrabHandleCenter();
    
    float aberration = 0.05f + grabHandleHoverAlpha_ * 0.10f;
    glm::vec3 grabHandleBaseColor = glm::vec3(1.0f);
    glm::vec3 grabHandleAbColor1 = glm::vec3(0.93f, 0.72f, 1.00f);
    glm::vec3 grabHandleAbColor2 = glm::vec3(0.7f, 0.9f, 1.0f);

    // Correct billboarding for grab handle
    glm::mat4 cameraWorld = glm::inverse(view);
    glm::vec3 camRight = glm::normalize(glm::vec3(cameraWorld[0]));
    glm::vec3 camUp    = glm::normalize(glm::vec3(cameraWorld[1]));
    glm::vec3 camFwd   = glm::normalize(glm::vec3(cameraWorld[2]));

    glm::mat4 grabHandleModel = glm::translate(glm::mat4(1.0f), grabHandleCenter) *
                           glm::mat4(glm::mat3(camRight, camUp, camFwd)) *
                           glm::scale(glm::mat4(1.0f), glm::vec3(grabHandleRadius * 2.0f));
    renderer.RenderVRMenuWidget(view, projection, grabHandleModel, grabHandleBaseColor, aberration, alpha, grabHandleAbColor1, grabHandleAbColor2);

    textRenderer.SetPanelModelMatrix(transform);

    // Render all widgets on this panel
    for (auto& widget : widgets_) {
        widget->Render(renderer, textRenderer, transform, view, projection, alpha);
    }
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
    if (isHoveringGrabHandle) {
        isGrabbing = true;
        return true;
    }
    if (hoveredWidget_) { hoveredWidget_->HandleClick(); return true; }
    return false;
}

glm::vec3 VRPanel::GetGrabHandleCenter() {
    float panelLocalScale = glm::length(glm::vec3(transform[0]));
    glm::vec3 panelOrigin = glm::vec3(transform[3]);
    glm::vec3 panelUp = glm::normalize(glm::vec3(transform[1]));
    glm::vec3 panelRight = glm::normalize(glm::vec3(transform[0]));
    float displayHeight = 0.05f * panelLocalScale;
    glm::vec3 displayCenter = panelOrigin + panelUp * (0.06f * panelLocalScale);
    float grabHandleRadius = 0.012f * panelLocalScale;
    glm::vec3 baseTopPosition = displayCenter + panelUp * (displayHeight * 0.5f + grabHandleRadius + 0.01f * panelLocalScale);
    float spacing = 0.04f * panelLocalScale;
    return baseTopPosition + panelRight * (-1.5f * spacing);
}

}

