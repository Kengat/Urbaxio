#define GLM_ENABLE_EXPERIMENTAL
#include "ui/VRConfirmButtonWidget.h"
#include "renderer.h"
#include <glm/gtx/intersect.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/norm.hpp>

namespace Urbaxio::UI {

VRConfirmButtonWidget::VRConfirmButtonWidget(const glm::vec3& localPos, float diameter, glm::vec3 color, std::function<void()> onClick)
    : localPosition_(localPos), diameter_(diameter), color_(color), textureId_(0), onClick_(onClick) {}

VRConfirmButtonWidget::VRConfirmButtonWidget(const glm::vec3& localPos, float diameter, unsigned int textureId, std::function<void()> onClick)
    : localPosition_(localPos), diameter_(diameter), color_(1.0f), textureId_(textureId), onClick_(onClick) {}

void VRConfirmButtonWidget::Update(const Ray& localRay, bool isClicked) {
    const float FADE_SPEED = 0.15f;
    float targetAlpha = isHovered_ ? 1.0f : 0.0f;
    hoverAlpha_ += (targetAlpha - hoverAlpha_) * FADE_SPEED;
}

void VRConfirmButtonWidget::HandleClick() {
    if (isHovered_ && onClick_) {
        onClick_();
    }
}

void VRConfirmButtonWidget::Render(Urbaxio::Renderer& renderer, Urbaxio::TextRenderer& textRenderer, const glm::mat4& panelTransform, const glm::mat4& view, const glm::mat4& projection, float alpha) {
    float panelLocalScale = glm::length(glm::vec3(panelTransform[0]));
    float scaledDiameter = diameter_ * panelLocalScale;

    glm::vec3 worldPos = panelTransform * glm::vec4(localPosition_, 1.0f);

    // Correct billboarding
    glm::mat4 cameraWorld = glm::inverse(view);
    glm::vec3 camRight = glm::normalize(glm::vec3(cameraWorld[0]));
    glm::vec3 camUp    = glm::normalize(glm::vec3(cameraWorld[1]));
    glm::vec3 camFwd   = glm::normalize(glm::vec3(cameraWorld[2]));
    glm::mat4 modelMatrix = glm::translate(glm::mat4(1.0f), worldPos) * glm::mat4(glm::mat3(camRight, camUp, camFwd)) * glm::scale(glm::mat4(1.0f), glm::vec3(scaledDiameter));
    
    float aberration = 0.05f + hoverAlpha_ * 0.10f; // Smoothly interpolates from 0.05 to 0.15

    glm::vec3 abColor1 = color_;
    glm::vec3 abColor2 = color_;

    const glm::vec3 greenColor(0.1f, 0.8f, 0.2f);
    const glm::vec3 whiteColor(1.0f, 1.0f, 1.0f);

    if (glm::distance2(color_, greenColor) < 0.01f) {
        abColor1 = glm::vec3(0.55f, 1.00f, 0.18f);
        abColor2 = glm::vec3(0.00f, 1.00f, 0.75f);
    } else if (glm::distance2(color_, whiteColor) < 0.01f) {
        abColor1 = glm::vec3(0.93f, 0.72f, 1.00f);
        abColor2 = glm::vec3(0.7f, 0.9f, 1.0f);
    }
    
    // Always render the sphere as a background
    renderer.RenderVRMenuWidget(view, projection, modelMatrix, color_, aberration, alpha, abColor1, abColor2);

    // If there's a texture, render it on top
    if (textureId_ != 0) {
        // NEW: Move the icon slightly forward from the panel's surface to prevent z-fighting
        glm::vec3 panelForward = glm::normalize(glm::vec3(panelTransform[2]));
        glm::vec3 iconWorldPos = worldPos + panelForward * 0.001f; // A small offset

        // Slightly smaller model matrix for the icon to create a border effect
        glm::mat4 iconModelMatrix = glm::translate(glm::mat4(1.0f), iconWorldPos) * glm::mat4(glm::mat3(camRight, camUp, camFwd)) * glm::scale(glm::mat4(1.0f), glm::vec3(scaledDiameter * 0.8f));
        renderer.RenderVRMenuWidget(view, projection, iconModelMatrix, glm::vec3(1.0f), 0.0f, alpha, glm::vec3(0.0f), glm::vec3(0.0f), textureId_);
    }
}

HitResult VRConfirmButtonWidget::CheckIntersection(const Ray& localRay) {
    HitResult result;
    float t;
    float radius = diameter_ * 0.5f;

    if (glm::intersectRaySphere(localRay.origin, localRay.direction, localPosition_, radius * radius, t) && t > 0) {
        result.didHit = true;
        result.distance = t;
        result.hitWidget = this;
    }
    return result;
}

} // namespace Urbaxio::UI

