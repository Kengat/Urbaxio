#define GLM_ENABLE_EXPERIMENTAL
#include "ui/VRSliderWidget.h"
#include "renderer.h"
#include "TextRenderer.h"
#include <glm/gtx/intersect.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>

namespace Urbaxio::UI {

VRSliderWidget::VRSliderWidget(const std::string& label, const glm::vec3& localPos, const glm::vec2& size, 
                               float minVal, float maxVal, float step, float initialVal, 
                               std::function<void(float)> onValueChanged)
    : label_(label), localPosition_(localPos), size_(size),
      minVal_(minVal), maxVal_(maxVal), step_(step), value_(initialVal),
      onValueChanged_(onValueChanged)
{
    // Clamp initial value
    value_ = std::clamp(value_, minVal_, maxVal_);
}

void VRSliderWidget::Update(const Ray& localRay, bool triggerPressed, bool triggerReleased, bool triggerHeld, bool aButtonPressed, float stickY) {
    const float FADE_SPEED = 0.15f;
    float targetAlpha = (isHovered_ || isDragging_) ? 1.0f : 0.0f;
    hoverAlpha_ += (targetAlpha - hoverAlpha_) * FADE_SPEED;

    // Start dragging
    if (isHovered_ && triggerPressed) {
        isDragging_ = true;
    }

    // Stop dragging
    if (!triggerHeld) {
        isDragging_ = false;
    }

    // Process Dragging
    if (isDragging_) {
        // Intersect ray with the plane of the slider (Z = localPosition.z)
        float t;
        if (glm::intersectRayPlane(localRay.origin, localRay.direction, localPosition_, glm::vec3(0, 0, 1), t) && t > 0) {
            glm::vec3 hitPoint = localRay.origin + localRay.direction * t;
            
            // Calculate normalized position (0.0 to 1.0) along the width of the slider
            // Local X range is [pos.x - width/2, pos.x + width/2]
            float localX = hitPoint.x - localPosition_.x;
            float halfWidth = size_.x * 0.5f;
            
            // Add a small padding for the handle radius so we can reach min/max easily
            float handleRadius = size_.y * 0.5f; 
            float trackWidth = size_.x - handleRadius * 2.0f;
            
            float normalizedPos = (localX + trackWidth * 0.5f) / trackWidth;
            normalizedPos = std::clamp(normalizedPos, 0.0f, 1.0f);
            
            // Map to value range
            float rawValue = minVal_ + normalizedPos * (maxVal_ - minVal_);
            
            if (step_ > 0.0001f) {
                // Snap to steps
                float steps = std::round((rawValue - minVal_) / step_);
                value_ = minVal_ + steps * step_;
            } else {
                value_ = rawValue;
            }
            
            // Clamp result
            value_ = std::clamp(value_, minVal_, maxVal_);
            
            if (onValueChanged_) {
                onValueChanged_(value_);
            }
        }
    }
}

void VRSliderWidget::Render(Urbaxio::Renderer& renderer, Urbaxio::TextRenderer& textRenderer, const glm::mat4& panelTransform, const glm::mat4& view, const glm::mat4& projection, float alpha, const std::optional<MaskData>& mask) const {
    float panelLocalScale = glm::length(glm::vec3(panelTransform[0]));
    
    // 1. Render Track (Pill shape background)
    glm::mat4 trackModel = panelTransform * 
                           glm::translate(glm::mat4(1.0f), localPosition_) * 
                           glm::scale(glm::mat4(1.0f), glm::vec3(size_.x, size_.y * 0.3f, 1.0f)); // Thinner track
    
    glm::vec3 trackColor = glm::vec3(0.2f, 0.2f, 0.25f);
    // --- MODIFIED: Pass 'mask' to RenderVRPanel ---
    renderer.RenderVRPanel(view, projection, trackModel, trackColor, 0.5f, alpha, mask);
    // ----------------------------------------------

    // 2. Render Handle (Sphere)
    float handleDiameter = size_.y * 0.8f;
    float trackUsableWidth = size_.x - handleDiameter;
    float normalizedValue = (value_ - minVal_) / (maxVal_ - minVal_);
    float handleXOffset = (normalizedValue - 0.5f) * trackUsableWidth;
    
    glm::vec3 handleLocalPos = localPosition_ + glm::vec3(handleXOffset, 0.0f, 0.005f); // Slightly raised
    glm::vec3 handleWorldPos = panelTransform * glm::vec4(handleLocalPos, 1.0f);

    // Billboard math for sphere
    glm::vec3 cameraPos = renderer.getCyclopsEyePosition();
    glm::vec3 cameraUp = glm::inverse(view)[1];
    glm::mat4 lookAtMatrix = glm::lookAt(handleWorldPos, cameraPos, cameraUp);
    glm::mat4 rotationMatrix = glm::inverse(lookAtMatrix);
    rotationMatrix[3] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);

    glm::mat4 handleModel = glm::translate(glm::mat4(1.0f), handleWorldPos) *
                            rotationMatrix *
                            glm::scale(glm::mat4(1.0f), glm::vec3(handleDiameter * panelLocalScale));

    // Handle Highlight
    glm::vec3 baseColor = isDragging_ ? glm::vec3(1.0f, 0.8f, 0.2f) : glm::vec3(0.7f, 0.7f, 0.75f);
    glm::vec3 ab1 = isDragging_ ? glm::vec3(1.0f, 0.5f, 0.0f) : glm::vec3(0.5f, 0.5f, 1.0f);
    glm::vec3 ab2 = isDragging_ ? glm::vec3(1.0f, 1.0f, 0.0f) : glm::vec3(0.0f, 1.0f, 1.0f);
    float aberration = 0.05f + hoverAlpha_ * 0.15f;

    renderer.RenderVRMenuWidget(view, projection, handleModel, baseColor, aberration, alpha, ab1, ab2, 0, mask);

    // 3. Render Text
    textRenderer.SetPanelModelMatrix(panelTransform);
    
    // Label (Top Left)
    glm::vec3 labelPos = localPosition_ + glm::vec3(-size_.x * 0.5f, size_.y * 0.7f, 0.0f);
    textRenderer.AddTextOnPanel(label_, labelPos, glm::vec4(1.0f, 1.0f, 1.0f, alpha), size_.y * 0.5f, Urbaxio::TextAlign::LEFT, mask);

    // Value (Top Right)
    std::stringstream ss;
    if (step_ >= 1.0f || (std::abs(step_) < 1e-5 && std::abs(std::round(value_) - value_) < 1e-5)) {
        ss << (int)std::round(value_);
    } else {
        ss << std::fixed << std::setprecision(2) << value_;
    }
    
    // Estimate width to align right
    glm::vec2 valTextSize = textRenderer.GetTextSize(ss.str(), size_.y * 0.5f);
    glm::vec3 valuePos = localPosition_ + glm::vec3(size_.x * 0.5f - valTextSize.x, size_.y * 0.7f, 0.0f);
    textRenderer.AddTextOnPanel(ss.str(), valuePos, glm::vec4(0.7f, 0.9f, 1.0f, alpha), size_.y * 0.5f, Urbaxio::TextAlign::LEFT, mask);
}

HitResult VRSliderWidget::CheckIntersection(const Ray& localRay) {
    HitResult result;
    float t;
    // Box intersection for the entire slider area including label space
    if (glm::intersectRayPlane(localRay.origin, localRay.direction, localPosition_, glm::vec3(0, 0, 1), t) && t > 0) {
        glm::vec3 hitPoint = localRay.origin + localRay.direction * t;
        
        // Hitbox is slightly taller to encompass the label
        float hitHeight = size_.y * 2.0f; 
        
        if (glm::abs(hitPoint.x - localPosition_.x) <= size_.x * 0.5f &&
            glm::abs(hitPoint.y - (localPosition_.y + size_.y * 0.25f)) <= hitHeight * 0.5f) {
            result.didHit = true;
            result.distance = t;
            result.hitWidget = this;
        }
    }
    return result;
}

void VRSliderWidget::HandleClick() {
    // Click handling is done in Update for sliders
}

void VRSliderWidget::SetLocalPosition(const glm::vec3& pos) { localPosition_ = pos; }
const glm::vec3& VRSliderWidget::GetLocalPosition() const { return localPosition_; }
void VRSliderWidget::SetSize(const glm::vec2& size) { size_ = size; }
glm::vec2 VRSliderWidget::GetSize() const { return {size_.x, size_.y * 1.5f}; } // Report larger height for layout to account for label

} // namespace Urbaxio::UI

