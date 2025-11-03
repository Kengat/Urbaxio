#include "ui/VRDisplayWidget.h"
#include "renderer.h"
#include "TextRenderer.h"
#include <glm/gtc/matrix_transform.hpp>

namespace Urbaxio::UI {

VRDisplayWidget::VRDisplayWidget(const glm::vec3& localPos, const glm::vec2& size, std::string& textValue)
    : localPosition_(localPos), size_(size), textValue_(textValue) {}

void VRDisplayWidget::Update(const Ray& localRay, bool isClicked) {
    // Display is not interactive
}

void VRDisplayWidget::Render(Urbaxio::Renderer& renderer, Urbaxio::TextRenderer& textRenderer, const glm::mat4& panelTransform, const glm::mat4& view, const glm::mat4& projection, float alpha) {
    float panelLocalScale = glm::length(glm::vec3(panelTransform[0]));

    // 1. Render Background Panel (copied from old code)
    // The panel itself is already scaled by panelTransform. We just need to apply local size.
    glm::mat4 displayModel = panelTransform * 
                             glm::translate(glm::mat4(1.0f), localPosition_) * 
                             glm::scale(glm::mat4(1.0f), glm::vec3(size_.x, size_.y, 1.0f));

    renderer.RenderVRPanel(view, projection, displayModel, glm::vec3(0.1f, 0.15f, 0.25f), 0.2f, 0.7f * alpha);

    // 2. Render Text on top (flat on panel)
    // The panel's world transform is already set in TextRenderer by VRPanel::Render
    float textHeight = 0.03f; // Height is in local panel units
    textRenderer.AddTextOnPanel(textValue_, localPosition_, glm::vec4(1.0f, 1.0f, 1.0f, alpha), textHeight);
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

