#define GLM_ENABLE_EXPERIMENTAL
#include "ui/VRToolButtonWidget.h"
#include "renderer.h"
#include "TextRenderer.h"
#include <glm/gtx/intersect.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace Urbaxio::UI {

VRToolButtonWidget::VRToolButtonWidget(const std::string& text, const glm::vec3& localPos, const glm::vec2& size,
                                       GLuint textureId,
                                       Urbaxio::Tools::ToolType toolType, Urbaxio::Tools::ToolManager& toolManager,
                                       std::function<void()> onClick)
    : text_(text), localPosition_(localPos), size_(size), 
      textureId_(textureId),
      toolType_(toolType), toolManager_(toolManager), onClick_(onClick) {}

void VRToolButtonWidget::Update(const Ray& localRay, bool isClicked, bool isClickReleased, float stickY) {
    const float FADE_SPEED = 0.15f;
    float targetAlpha = isHovered_ ? 1.0f : 0.0f;
    sphereHoverAlpha_ += (targetAlpha - sphereHoverAlpha_) * FADE_SPEED;
    textHoverAlpha_ += (targetAlpha - textHoverAlpha_) * FADE_SPEED;
}

void VRToolButtonWidget::HandleClick() {
    if (isHovered_ && onClick_) {
        onClick_();
    }
}

HitResult VRToolButtonWidget::CheckIntersection(const Ray& localRay) {
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

void VRToolButtonWidget::Render(Urbaxio::Renderer& renderer, Urbaxio::TextRenderer& textRenderer, const glm::mat4& panelTransform, const glm::mat4& view, const glm::mat4& projection, float alpha, const std::optional<MaskData>& mask) const {
    bool isSelected = toolManager_.GetActiveToolType() == toolType_;
    float panelLocalScale = glm::length(glm::vec3(panelTransform[0]));

    float sphereDiameter = size_.y * 0.8f;
    glm::vec3 sphereLocalPos = localPosition_; 
    glm::vec3 sphereWorldPos = panelTransform * glm::vec4(sphereLocalPos, 1.0f);

    glm::mat4 cameraWorld = glm::inverse(view);
    glm::vec3 camRight = glm::normalize(glm::vec3(cameraWorld[0]));
    glm::vec3 camUp    = glm::normalize(glm::vec3(cameraWorld[1]));
    glm::vec3 camFwd   = glm::normalize(glm::vec3(cameraWorld[2]));
    glm::mat4 sphereModel = glm::translate(glm::mat4(1.0f), sphereWorldPos) *
                            glm::mat4(glm::mat3(camRight, camUp, glm::vec3(0))) *
                            glm::scale(glm::mat4(1.0f), glm::vec3(sphereDiameter * panelLocalScale));

    float aberration = 0.05f + sphereHoverAlpha_ * 0.10f;
    
    glm::vec3 baseColor = isSelected ? selectedColor_ : inactiveColor_;
    glm::vec3 abColor1 = isSelected ? orange_aberration1_ : blue_aberration1_;
    glm::vec3 abColor2 = isSelected ? orange_aberration2_ : blue_aberration2_;

    renderer.RenderVRMenuWidget(view, projection, sphereModel, baseColor, aberration, alpha, abColor1, abColor2);

    if (textureId_ != 0) {
        const float ICON_FORWARD_FACTOR = 0.15f; // Пропорционально диаметру
        float scaledSphereDiameter = sphereDiameter * panelLocalScale;
        glm::vec3 iconWorldPos = sphereWorldPos + camFwd * (scaledSphereDiameter * ICON_FORWARD_FACTOR);
        glm::mat4 iconModel = glm::translate(glm::mat4(1.0f), iconWorldPos) *
                              glm::mat4(glm::mat3(camRight, camUp, glm::vec3(0))) *
                              glm::scale(glm::mat4(1.0f), glm::vec3(scaledSphereDiameter * 0.8f));

        renderer.RenderVRMenuWidget(view, projection, iconModel, glm::vec3(1.0f), 0.0f, alpha, glm::vec3(0.0f), glm::vec3(0.0f), textureId_);
    }

    if (textHoverAlpha_ > 0.01f) {
        float textHeight = size_.y * 0.6f;
        glm::vec4 textColor = glm::vec4(1.0f, 1.0f, 1.0f, alpha * textHoverAlpha_);

        float slideOffset = 0.015f;
        glm::vec3 textLocalPos = localPosition_;
        textLocalPos.x += (sphereDiameter * 0.7f);
        textLocalPos.x -= slideOffset * (1.0f - textHoverAlpha_);

        textRenderer.AddTextOnPanel(text_, textLocalPos, textColor, textHeight, Urbaxio::TextAlign::LEFT, mask);
    }
}

void VRToolButtonWidget::SetLocalPosition(const glm::vec3& pos) { localPosition_ = pos; }

const glm::vec3& VRToolButtonWidget::GetLocalPosition() const { return localPosition_; }

glm::vec2 VRToolButtonWidget::GetSize() const { return size_; }

void VRToolButtonWidget::SetSize(const glm::vec2& size) { size_ = size; }

} // namespace Urbaxio::UI

