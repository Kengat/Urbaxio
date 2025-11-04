#include "ui/VRDisplayWidget.h"
#include "renderer.h"
#include "TextRenderer.h"
#include <glm/gtc/matrix_transform.hpp>

namespace Urbaxio::UI {

VRDisplayWidget::VRDisplayWidget(const glm::vec3& localPos, const glm::vec2& size, std::string& textValue)
    : localPosition_(localPos), size_(size), textValue_(textValue) {}

void VRDisplayWidget::Update(const Ray& localRay, bool triggerPressed, bool triggerReleased, bool triggerHeld, bool aButtonPressed, float stickY) {
    // Display is not interactive
}

void VRDisplayWidget::Render(Urbaxio::Renderer& renderer, Urbaxio::TextRenderer& textRenderer, const glm::mat4& panelTransform, const glm::mat4& view, const glm::mat4& projection, float alpha, const std::optional<MaskData>& mask) const {
    float panelLocalScale = glm::length(glm::vec3(panelTransform[0]));

    glm::mat4 displayModel = panelTransform * 
                             glm::translate(glm::mat4(1.0f), localPosition_) * 
                             glm::scale(glm::mat4(1.0f), glm::vec3(size_.x, size_.y, 1.0f));

    renderer.RenderVRPanel(view, projection, displayModel, glm::vec3(0.1f, 0.15f, 0.25f), 0.2f, 0.7f * alpha);

    float textHeight = 0.03f;
    textRenderer.AddTextOnPanel(textValue_, localPosition_, glm::vec4(1.0f, 1.0f, 1.0f, alpha), textHeight, Urbaxio::TextAlign::CENTER, mask);
}

HitResult VRDisplayWidget::CheckIntersection(const Ray& localRay) {
    return {};
}

void VRDisplayWidget::HandleClick() {
    // Display is not interactive
}

void VRDisplayWidget::SetLocalPosition(const glm::vec3& pos) { localPosition_ = pos; }

const glm::vec3& VRDisplayWidget::GetLocalPosition() const { return localPosition_; }

glm::vec2 VRDisplayWidget::GetSize() const { return size_; }

} // namespace Urbaxio::UI

