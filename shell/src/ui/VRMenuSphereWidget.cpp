#define GLM_ENABLE_EXPERIMENTAL
#include "ui/VRMenuSphereWidget.h"
#include "renderer.h"
#include <glm/gtx/intersect.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace Urbaxio::UI {

VRMenuSphereWidget::VRMenuSphereWidget(const glm::vec3& localPos, float diameter, 
                                       const glm::vec3& baseColor, const glm::vec3& abColor1, const glm::vec3& abColor2, 
                                       std::function<void()> onClick)
    : localPosition_(localPos), diameter_(diameter), 
      baseColor_(baseColor), aberrationColor1_(abColor1), aberrationColor2_(abColor2),
      onClick_(onClick) {}

void VRMenuSphereWidget::Update(const Ray& localRay, bool isClicked, bool isClickReleased, float stickY) {
    const float FADE_SPEED = 0.15f;
    float targetAlpha = isHovered_ ? 1.0f : 0.0f;
    hoverAlpha_ += (targetAlpha - hoverAlpha_) * FADE_SPEED;
}

void VRMenuSphereWidget::HandleClick() {
    if (isHovered_ && onClick_) {
        onClick_();
    }
}

void VRMenuSphereWidget::Render(Urbaxio::Renderer& renderer, Urbaxio::TextRenderer& textRenderer, const glm::mat4& panelTransform, const glm::mat4& view, const glm::mat4& projection, float alpha, const std::optional<MaskData>& mask) const {
    float panelLocalScale = glm::length(glm::vec3(panelTransform[0]));
    float scaledDiameter = diameter_ * panelLocalScale;

    glm::vec3 worldPos = panelTransform * glm::vec4(localPosition_, 1.0f);

    glm::mat4 cameraWorld = glm::inverse(view);
    glm::vec3 camRight = glm::normalize(glm::vec3(cameraWorld[0]));
    glm::vec3 camUp    = glm::normalize(glm::vec3(cameraWorld[1]));
    glm::vec3 camFwd   = glm::normalize(glm::vec3(cameraWorld[2]));
    glm::mat4 modelMatrix = glm::translate(glm::mat4(1.0f), worldPos) * glm::mat4(glm::mat3(camRight, camUp, camFwd)) * glm::scale(glm::mat4(1.0f), glm::vec3(scaledDiameter));
    
    float aberration = 0.05f + hoverAlpha_ * 0.10f;

    renderer.RenderVRMenuWidget(view, projection, modelMatrix, baseColor_, aberration, alpha, aberrationColor1_, aberrationColor2_);
}

HitResult VRMenuSphereWidget::CheckIntersection(const Ray& localRay) {
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

void VRMenuSphereWidget::SetLocalPosition(const glm::vec3& pos) { localPosition_ = pos; }

const glm::vec3& VRMenuSphereWidget::GetLocalPosition() const { return localPosition_; }

glm::vec2 VRMenuSphereWidget::GetSize() const { return {diameter_, diameter_}; }

} // namespace Urbaxio::UI

