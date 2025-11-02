// shell/src/vr_ui/VRBillboardButton.cpp
#define GLM_ENABLE_EXPERIMENTAL
#include "vr_ui/VRBillboardButton.h"
#include "renderer.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/intersect.hpp>

namespace Urbaxio::VRUI {

VRBillboardButton::VRBillboardButton(const glm::mat4& localTransform, float radius)
    : VRWidget(localTransform), radius_(radius) {}

void VRBillboardButton::Update(const VRInputState& input, const VRUIStyle& style) {
    if (isHovered_) {
        if (input.triggerClicked && OnClick) {
            OnClick();
        }
    }
    
    if (OnPressStateChanged) {
        if (isHovered_ && input.triggerClicked && !wasPressed_) {
            wasPressed_ = true;
            OnPressStateChanged(true);
        } else if (input.triggerReleased && wasPressed_) {
            wasPressed_ = false;
            OnPressStateChanged(false);
        }
    }

    float targetAberration = isHovered_ ? 0.15f : 0.05f;
    hoverAberration_ += (targetAberration - hoverAberration_) * 0.1f;
}

void VRBillboardButton::Render(Renderer& renderer, TextRenderer& textRenderer, const glm::mat4& view, const glm::mat4& projection, const VRUIStyle& style) {
    float panelScale = glm::length(glm::vec3(parentTransform_[0]));
    float scaledRadius = radius_ * panelScale;

    glm::mat4 cameraWorld = glm::inverse(view);
    glm::vec3 camRight = glm::normalize(glm::vec3(cameraWorld[0]));
    glm::vec3 camUp    = glm::normalize(glm::vec3(cameraWorld[1]));
    glm::vec3 camFwd   = glm::normalize(glm::vec3(cameraWorld[2]));

    glm::mat4 billboardModel = 
        glm::translate(glm::mat4(1.0f), glm::vec3(finalTransform_[3])) *
        glm::mat4(glm::mat3(camRight, camUp, camFwd)) *
        glm::scale(glm::mat4(1.0f), glm::vec3(scaledRadius * 2.0f));

    renderer.RenderVRMenuWidget(view, projection, billboardModel, baseColor, hoverAberration_, 1.0f, baseColor, baseColor);
}

float VRBillboardButton::CheckHit(const VRInputState& input) {
    float panelScale = glm::length(glm::vec3(parentTransform_[0]));
    glm::vec3 widgetCenter = glm::vec3(finalTransform_[3]);
    float dist;
    if (glm::intersectRaySphere(input.rayOrigin, input.rayDirection, widgetCenter, (radius_ * panelScale) * (radius_ * panelScale), dist)) {
        return dist;
    }
    return -1.0f;
}

}

