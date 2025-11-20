#define GLM_ENABLE_EXPERIMENTAL
#include "ui/VRToggleWidget.h"
#include "renderer.h"
#include "TextRenderer.h"
#include <glm/gtx/intersect.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>

namespace Urbaxio::UI {

VRToggleWidget::VRToggleWidget(const std::string& label, const glm::vec3& localPos, const glm::vec2& size, 
                               bool initialValue, std::function<void(bool)> onValueChanged)
    : label_(label), localPosition_(localPos), size_(size),
      value_(initialValue), onValueChanged_(onValueChanged)
{
    animT_ = value_ ? 1.0f : 0.0f;
}

void VRToggleWidget::Update(const Ray& localRay, bool triggerPressed, bool triggerReleased, bool triggerHeld, bool aButtonPressed, float stickY) {
    const float FADE_SPEED = 0.15f;
    float targetAlpha = isHovered_ ? 1.0f : 0.0f;
    hoverAlpha_ += (targetAlpha - hoverAlpha_) * FADE_SPEED;

    const float ANIM_SPEED = 0.2f;
    float targetT = value_ ? 1.0f : 0.0f;
    animT_ += (targetT - animT_) * ANIM_SPEED;
}

void VRToggleWidget::HandleClick() {
    if (isHovered_) {
        value_ = !value_;
        if (onValueChanged_) {
            onValueChanged_(value_);
        }
    }
}

void VRToggleWidget::Render(Urbaxio::Renderer& renderer, Urbaxio::TextRenderer& textRenderer, const glm::mat4& panelTransform, const glm::mat4& view, const glm::mat4& projection, float alpha, const std::optional<MaskData>& mask) const {
    float panelLocalScale = glm::length(glm::vec3(panelTransform[0]));
    
    // Layout dimensions
    float toggleWidth = size_.x * 0.4f; // Toggle occupies right side
    float toggleHeight = size_.y;
    float knobDiameter = toggleHeight * 0.85f;
    
    // 1. Render Text (Left side)
    textRenderer.SetPanelModelMatrix(panelTransform);
    glm::vec3 textPos = localPosition_ + glm::vec3(-size_.x * 0.5f, 0.0f, 0.0f);
    textRenderer.AddTextOnPanel(label_, textPos, glm::vec4(1.0f, 1.0f, 1.0f, alpha), size_.y * 0.7f, Urbaxio::TextAlign::LEFT, mask);

    // 2. Render Track (Pill shape) on Right side
    glm::vec3 trackLocalPos = localPosition_ + glm::vec3(size_.x * 0.5f - toggleWidth * 0.5f, 0.0f, 0.0f);
    
    glm::mat4 trackModel = panelTransform * 
                           glm::translate(glm::mat4(1.0f), trackLocalPos) * 
                           glm::scale(glm::mat4(1.0f), glm::vec3(toggleWidth, toggleHeight, 1.0f));
    
    // Track color changes slightly when on
    glm::vec3 offColor(0.2f, 0.2f, 0.25f);
    glm::vec3 onColor(0.1f, 0.4f, 0.15f); // Dark green
    glm::vec3 trackColor = glm::mix(offColor, onColor, animT_);
    
    // --- MODIFIED: Use 0.5f for full capsule rounding ---
    renderer.RenderVRPanel(view, projection, trackModel, trackColor, 0.5f, alpha); 
    // ----------------------------------------------------

    // 3. Render Knob (Sphere)
    float travelDist = toggleWidth - toggleHeight; // Distance knob moves
    float xOffset = (animT_ - 0.5f) * travelDist;
    
    glm::vec3 knobLocalPos = trackLocalPos + glm::vec3(xOffset, 0.0f, 0.005f);
    glm::vec3 knobWorldPos = panelTransform * glm::vec4(knobLocalPos, 1.0f);

    // Billboard math
    glm::vec3 cameraPos = renderer.getCyclopsEyePosition();
    glm::vec3 cameraUp = glm::inverse(view)[1];
    glm::mat4 lookAtMatrix = glm::lookAt(knobWorldPos, cameraPos, cameraUp);
    glm::mat4 rotationMatrix = glm::inverse(lookAtMatrix);
    rotationMatrix[3] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);

    glm::mat4 knobModel = glm::translate(glm::mat4(1.0f), knobWorldPos) *
                          rotationMatrix *
                          glm::scale(glm::mat4(1.0f), glm::vec3(knobDiameter * panelLocalScale));

    // Knob Color
    glm::vec3 knobOff(0.7f, 0.7f, 0.75f);
    glm::vec3 knobOn(0.2f, 1.0f, 0.3f); // Bright green
    glm::vec3 knobColor = glm::mix(knobOff, knobOn, animT_);
    
    float aberration = 0.05f + hoverAlpha_ * 0.15f;
    renderer.RenderVRMenuWidget(view, projection, knobModel, knobColor, aberration, alpha, 
                                knobColor + 0.2f, knobColor - 0.2f, 0, mask);
}

HitResult VRToggleWidget::CheckIntersection(const Ray& localRay) {
    HitResult result;
    float t;
    if (glm::intersectRayPlane(localRay.origin, localRay.direction, localPosition_, glm::vec3(0, 0, 1), t) && t > 0) {
        glm::vec3 hitPoint = localRay.origin + localRay.direction * t;
        
        // Check full width (label + toggle)
        if (glm::abs(hitPoint.x - localPosition_.x) <= size_.x * 0.5f &&
            glm::abs(hitPoint.y - localPosition_.y) <= size_.y * 0.5f) {
            result.didHit = true;
            result.distance = t;
            result.hitWidget = this;
        }
    }
    return result;
}

void VRToggleWidget::SetLocalPosition(const glm::vec3& pos) { localPosition_ = pos; }
const glm::vec3& VRToggleWidget::GetLocalPosition() const { return localPosition_; }
void VRToggleWidget::SetSize(const glm::vec2& size) { size_ = size; }
glm::vec2 VRToggleWidget::GetSize() const { return size_; }

} // namespace Urbaxio::UI

