#define GLM_ENABLE_EXPERIMENTAL
#include "ui/VRPanel.h"
#include "renderer.h"
#include "TextRenderer.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtx/intersect.hpp>
#include <limits>
#include <cmath>

namespace Urbaxio::UI {

VRPanel::VRPanel(const std::string& name, const glm::vec2& size, const glm::mat4& offsetTransform, float cornerRadius)
    : name_(name), size_(size), offsetTransform_(offsetTransform), cornerRadius_(cornerRadius), hoveredWidget_(nullptr), isGrabbing(false), isHoveringGrabHandle(false) {
    
    // Automatically create and position the resize handle
    glm::vec3 handlePos(size_.x * 0.5f, size_.y * 0.5f, 0.01f);
    resizeHandle_ = std::make_unique<VRConfirmButtonWidget>(handlePos, 0.02f, glm::vec3(1.0f), nullptr);
}

void VRPanel::AddWidget(std::unique_ptr<IVRWidget> widget) {
    widgets_.push_back(std::move(widget));
}

void VRPanel::SetLayout(std::unique_ptr<ILayout> layout) {
    layout_ = std::move(layout);
}

void VRPanel::RecalculateLayout() {
    if (layout_) {
        layout_->Apply(widgets_, size_);
    }
}

void VRPanel::Update(const Ray& worldRay, const glm::mat4& parentTransform, bool isClicked, bool isClickReleased, bool aButtonIsPressed) {
    if (!isGrabbing) {
        transform = parentTransform * offsetTransform_;
    }

    // --- Resize/Proportion Drag Logic ---
    if (isResizing_ || isChangingProportions_) {
        // Project the current controller position onto the panel's plane
        glm::vec3 panelOrigin = panelCenterAtResizeStart_;
        glm::vec3 panelNormal = -glm::normalize(glm::vec3(transform[2]));
        
        float t;
        glm::intersectRayPlane(worldRay.origin, worldRay.direction, panelOrigin, panelNormal, t);
        glm::vec3 controllerPosOnPlane = worldRay.origin + worldRay.direction * t;

        // Decompose the original offset transform to apply new scale
        glm::vec3 scale, translation, skew;
        glm::quat orientation;
        glm::vec4 perspective;
        (void)glm::decompose(offsetTransform_, scale, orientation, translation, skew, perspective);

        if (isResizing_) {
            // --- UNIFORM SCALE (старая логика) ---
            float currentDist = glm::distance(panelCenterAtResizeStart_, controllerPosOnPlane);
            float initialDist = glm::distance(panelCenterAtResizeStart_, resizeStartControllerPos_);
            float scaleFactor = (initialDist > 0.01f) ? (currentDist / initialDist) : 1.0f;
            scaleFactor = glm::max(0.1f, scaleFactor);
            glm::vec3 newScale = resizeStartScale_ * scaleFactor;
            
            offsetTransform_ = glm::translate(glm::mat4(1.0f), translation) * glm::mat4_cast(orientation) * glm::scale(glm::mat4(1.0f), newScale);
        
        } else { // isChangingProportions_
            // --- NON-UNIFORM SCALE (новая логика) ---
            glm::vec3 currentVec = controllerPosOnPlane - panelCenterAtResizeStart_;
            glm::vec3 initialVec = resizeStartControllerPos_ - panelCenterAtResizeStart_;

            float currentX = glm::dot(currentVec, initialPanelXDir_);
            float currentY = glm::dot(currentVec, initialPanelYDir_);
            float initialX = glm::dot(initialVec, initialPanelXDir_);
            float initialY = glm::dot(initialVec, initialPanelYDir_);

            float scaleX = (std::abs(initialX) > 0.01f) ? (currentX / initialX) : 1.0f;
            float scaleY = (std::abs(initialY) > 0.01f) ? (currentY / initialY) : 1.0f;

            glm::vec3 newScale = resizeStartScale_ * glm::vec3(std::abs(scaleX), std::abs(scaleY), 1.0f);
            
            offsetTransform_ = glm::translate(glm::mat4(1.0f), translation) * glm::mat4_cast(orientation) * glm::scale(glm::mat4(1.0f), newScale);
        }
    }

    if (isClickReleased) {
        isResizing_ = false;
        isChangingProportions_ = false;
    }

    // --- Interaction Logic (Hover, Click) ---
    glm::mat4 invTransform = glm::inverse(transform);
    Ray localRay;
    localRay.origin = invTransform * glm::vec4(worldRay.origin, 1.0f);
    localRay.direction = glm::normalize(glm::vec3(invTransform * glm::vec4(worldRay.direction, 0.0f)));

    bool clickConsumed = false;

    // First, check and update the resize handle
    HitResult resizeHit = resizeHandle_->CheckIntersection(localRay);
    resizeHandle_->SetHover(resizeHit.didHit);
    resizeHandle_->Update(localRay, false);

    if (resizeHit.didHit && isClicked) {
        // --- START RESIZE OR PROPORTION CHANGE ---
        clickConsumed = true;
        panelCenterAtResizeStart_ = glm::vec3(transform[3]);
        
        // Project start point onto panel plane
        glm::vec3 panelNormal = -glm::normalize(glm::vec3(transform[2]));
        float t;
        glm::intersectRayPlane(worldRay.origin, worldRay.direction, panelCenterAtResizeStart_, panelNormal, t);
        resizeStartControllerPos_ = worldRay.origin + worldRay.direction * t;

        // Store initial scale
        (void)glm::decompose(offsetTransform_, resizeStartScale_, glm::quat(), glm::vec3(), glm::vec3(), glm::vec4());

        if (aButtonIsPressed) {
            isChangingProportions_ = true;
            isResizing_ = false;
            // Store panel axes at start of drag
            initialPanelXDir_ = glm::normalize(glm::vec3(transform[0]));
            initialPanelYDir_ = glm::normalize(glm::vec3(transform[1]));
        } else {
            isResizing_ = true;
            isChangingProportions_ = false;
        }
    }
    
    // If not interacting with resize handle, check other widgets
    if (!clickConsumed) {
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
            if (hoveredWidget_) hoveredWidget_->SetHover(false);
            hoveredWidget_ = newHoveredWidget;
            if (hoveredWidget_) hoveredWidget_->SetHover(true);
        }
    } else {
        // If resize handle was clicked, ensure no other widget is hovered
        if (hoveredWidget_) hoveredWidget_->SetHover(false);
        hoveredWidget_ = nullptr;
    }
    
    for (auto& widget : widgets_) {
        widget->Update(localRay, false);
    }
}

void VRPanel::Render(Urbaxio::Renderer& renderer, Urbaxio::TextRenderer& textRenderer, const glm::mat4& view, const glm::mat4& projection) {
    if (!isVisible_ || alpha < 0.01f) return;

    // Render panel background
    glm::mat4 backgroundModel = transform * glm::scale(glm::mat4(1.0f), glm::vec3(size_.x, size_.y, 1.0f));
    renderer.RenderVRPanel(view, projection, backgroundModel, glm::vec3(0.43f, 0.65f, 0.82f), cornerRadius_, 0.25f * alpha);
    
    // Render resize handle
    if(resizeHandle_) {
        resizeHandle_->Render(renderer, textRenderer, transform, view, projection, alpha);
    }

    textRenderer.SetPanelModelMatrix(transform);
    for (auto& widget : widgets_) {
        widget->Render(renderer, textRenderer, transform, view, projection, alpha);
    }
    textRenderer.Render(view, projection);
}

HitResult VRPanel::CheckIntersection(const Ray& worldRay, const glm::mat4& parentTransform) {
    // This function is now only for checking the main panel area for hover, not widgets
    HitResult result;
    if (!isVisible_) return result;
    
    glm::mat4 finalTransform = parentTransform * offsetTransform_;
    
    // First, check the resize handle as it's on top
    glm::mat4 invFinalTransform = glm::inverse(finalTransform);
    Ray localRay;
    localRay.origin = invFinalTransform * glm::vec4(worldRay.origin, 1.0f);
    localRay.direction = glm::normalize(glm::vec3(invFinalTransform * glm::vec4(worldRay.direction, 0.0f)));
    HitResult handleHit = resizeHandle_->CheckIntersection(localRay);
    if(handleHit.didHit) {
        handleHit.distance = glm::distance(worldRay.origin, glm::vec3(finalTransform * glm::vec4(localRay.origin + localRay.direction * handleHit.distance, 1.0)));
        return handleHit;
    }

    // If handle not hit, check the main panel plane
    glm::vec3 panelOrigin = glm::vec3(finalTransform[3]);
    glm::vec3 panelNormal = -glm::normalize(glm::vec3(finalTransform[2]));
    
    float t;
    if (glm::intersectRayPlane(worldRay.origin, worldRay.direction, panelOrigin, panelNormal, t) && t > 0) {
        glm::vec3 hitPoint = worldRay.origin + worldRay.direction * t;
        glm::vec3 localHitPoint = invFinalTransform * glm::vec4(hitPoint, 1.0f);
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

bool VRPanel::IsResizing() const {
    return isResizing_;
}

bool VRPanel::IsChangingProportions() const {
    return isChangingProportions_;
}

}

