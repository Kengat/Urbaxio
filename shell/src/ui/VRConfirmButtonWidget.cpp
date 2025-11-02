#define GLM_ENABLE_EXPERIMENTAL
#include "ui/VRConfirmButtonWidget.h"
#include "renderer.h"
#include <glm/gtx/intersect.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace Urbaxio::UI {

VRConfirmButtonWidget::VRConfirmButtonWidget(const glm::vec3& localPos, float diameter, glm::vec3 color, std::function<void()> onClick)
    : localPosition_(localPos), diameter_(diameter), color_(color), onClick_(onClick) {}

void VRConfirmButtonWidget::Update(const Ray& localRay, bool isClicked) {
    // Click handling is now done in HandleClick
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
    
    float aberration = isHovered_ ? 0.15f : 0.05f;

    renderer.RenderVRMenuWidget(view, projection, modelMatrix, color_, aberration, alpha, color_, color_);
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

