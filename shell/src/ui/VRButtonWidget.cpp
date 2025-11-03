#define GLM_ENABLE_EXPERIMENTAL
#include "ui/VRButtonWidget.h"
#include "renderer.h"
#include "TextRenderer.h"
#include <glm/gtx/intersect.hpp>

namespace Urbaxio::UI {

VRButtonWidget::VRButtonWidget(const std::string& text, const glm::vec3& localPos, const glm::vec2& size, std::function<void()> onClick)
    : text_(text), localPosition_(localPos), size_(size), onClick_(onClick) {}

void VRButtonWidget::Update(const Ray& localRay, bool isClicked) {
    // Click logic moved to HandleClick, Update is for continuous state changes like animations.
}

void VRButtonWidget::Render(Urbaxio::Renderer& renderer, Urbaxio::TextRenderer& textRenderer, const glm::mat4& panelTransform, const glm::mat4& view, const glm::mat4& projection, float alpha) {
    float panelLocalScale = glm::length(glm::vec3(panelTransform[0]));
    glm::vec3 worldPos = panelTransform * glm::vec4(localPosition_, 1.0f);
    float textHeight = size_.y * panelLocalScale; // The size here is the text height, now scaled

    glm::vec4 color = glm::vec4(1.0f, 1.0f, 1.0f, alpha * (isHovered_ ? 1.0f : 0.7f));
    textRenderer.AddText(text_, worldPos, color, textHeight, view); // Pass correct view matrix
}

HitResult VRButtonWidget::CheckIntersection(const Ray& localRay) {
    HitResult result;
    float t;
    if (glm::intersectRayPlane(localRay.origin, localRay.direction, localPosition_, glm::vec3(0, 0, 1), t) && t > 0) {
        glm::vec3 hitPoint = localRay.origin + localRay.direction * t;
        if (glm::abs(hitPoint.x - localPosition_.x) <= size_.x * 0.5f &&
            glm::abs(hitPoint.y - localPosition_.y) <= size_.y * 0.5f) {
            result.didHit = true;
            result.distance = t;
            result.hitWidget = this;

        }
    }
    return result;
}

void VRButtonWidget::HandleClick() {
    if (isHovered_ && onClick_) onClick_();
}

void VRButtonWidget::SetLocalPosition(const glm::vec3& pos) { localPosition_ = pos; }

const glm::vec3& VRButtonWidget::GetLocalPosition() const { return localPosition_; }

glm::vec2 VRButtonWidget::GetSize() const { return size_; }

}

