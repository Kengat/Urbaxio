#define GLM_ENABLE_EXPERIMENTAL
#include "ui/VRTextureWidget.h"
#include "renderer.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/intersect.hpp>

namespace Urbaxio::UI {

VRTextureWidget::VRTextureWidget(const glm::vec3& localPos, const glm::vec2& size)
    : localPosition_(localPos), size_(size) {}

void VRTextureWidget::SetTexture(GLuint textureId) {
    textureId_ = textureId;
}

void VRTextureWidget::Update(const Ray& localRay, bool triggerPressed, bool triggerReleased, bool triggerHeld, bool aButtonPressed, float stickY) {
    // Passive widget for now
}

void VRTextureWidget::Render(Urbaxio::Renderer& renderer, Urbaxio::TextRenderer& textRenderer, const glm::mat4& panelTransform, const glm::mat4& view, const glm::mat4& projection, float alpha, const std::optional<MaskData>& mask) const {
    if (textureId_ == 0) return;

    glm::mat4 model = panelTransform * 
                      glm::translate(glm::mat4(1.0f), localPosition_) * 
                      glm::scale(glm::mat4(1.0f), glm::vec3(size_.x, size_.y, 1.0f));

    // Use the special method for textured quads
    renderer.RenderVRTexture(view, projection, model, textureId_, alpha, mask);
}

HitResult VRTextureWidget::CheckIntersection(const Ray& localRay) {
    HitResult result;
    float t;
    // Plane intersection logic
    if (glm::intersectRayPlane(localRay.origin, localRay.direction, localPosition_, glm::vec3(0, 0, 1), t) && t > 0) {
        glm::vec3 hitPoint = localRay.origin + localRay.direction * t;
        if (glm::abs(hitPoint.x - localPosition_.x) <= size_.x * 0.5f &&
            glm::abs(hitPoint.y - localPosition_.y) <= size_.y * 0.5f) {
            result.didHit = true;
            result.distance = t;
            result.hitWidget = (void*)this;
        }
    }
    return result;
}

void VRTextureWidget::HandleClick() {}
void VRTextureWidget::SetLocalPosition(const glm::vec3& pos) { localPosition_ = pos; }
const glm::vec3& VRTextureWidget::GetLocalPosition() const { return localPosition_; }
void VRTextureWidget::SetSize(const glm::vec2& size) { size_ = size; }
glm::vec2 VRTextureWidget::GetSize() const { return size_; }

}

