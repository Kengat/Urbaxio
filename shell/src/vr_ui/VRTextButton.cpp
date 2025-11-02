// shell/src/vr_ui/VRTextButton.cpp
// --- FIX: Enable experimental GLM features ---
#define GLM_ENABLE_EXPERIMENTAL

#include "vr_ui/VRTextButton.h"
#include "renderer.h"
#include "TextRenderer.h"
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/intersect.hpp>
#include <cmath>
#include <glad/glad.h>

namespace Urbaxio::VRUI {

VRTextButton::VRTextButton(const std::string& label, const glm::mat4& localTransform, float radius)
    : VRWidget(localTransform), label_(label), radius_(radius) {}

void VRTextButton::Update(const VRInputState& input, const VRUIStyle& style) {
    if (isHovered_ && input.triggerClicked && OnClick) {
        OnClick();
    }

    // Smoothly animate hover alpha
    float targetAlpha = isHovered_ ? 1.0f : 0.7f;
    hoverAlpha_ += (targetAlpha - hoverAlpha_) * 0.15f;
}

void VRTextButton::Render(Renderer& renderer, TextRenderer& textRenderer, const glm::mat4& view, const glm::mat4& projection, const VRUIStyle& style) {
    glm::vec3 widgetCenter = glm::vec3(finalTransform_[3]);
    float panelScale = glm::length(glm::vec3(parentTransform_[0]));

    // --- Dynamic Text Sizing ---
    const float desiredPxHeight = 30.0f;
    int viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    float viewportH = (float)viewport[3];
    
    float fovY = 2.0f * atan(1.0f / projection[1][1]);
    
    float viewZ = -(view * glm::vec4(widgetCenter, 1.0f)).z;
    viewZ = std::max(0.1f, viewZ);
    
    float worldUnitsPerPixel = viewZ * tanf(fovY / 2.0f) * 2.0f / viewportH;
    float textWorldSize = desiredPxHeight * worldUnitsPerPixel * panelScale;

    glm::vec4 finalColor = glm::vec4(style.textColor.r, style.textColor.g, style.textColor.b, hoverAlpha_);
    textRenderer.AddText(label_, widgetCenter, finalColor, textWorldSize, view);
}

float VRTextButton::CheckHit(const VRInputState& input) {
    float panelScale = glm::length(glm::vec3(parentTransform_[0]));
    glm::vec3 widgetCenter = glm::vec3(finalTransform_[3]);
    float dist;
    if (glm::intersectRaySphere(input.rayOrigin, input.rayDirection, widgetCenter, (radius_ * panelScale) * (radius_ * panelScale), dist)) {
        return dist;
    }
    return -1.0f;
}

} // namespace Urbaxio::VRUI

