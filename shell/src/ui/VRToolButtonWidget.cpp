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

    // -- START OF MODIFICATION --

    // Spherical billboard logic using camera's UP vector to prevent flipping.

    glm::vec3 cameraPos = renderer.getCyclopsEyePosition();

    glm::vec3 cameraUp = glm::inverse(view)[1]; // Get camera's up vector in world space

    glm::mat4 lookAtMatrix = glm::lookAt(sphereWorldPos, cameraPos, cameraUp);

    glm::mat4 rotationMatrix = glm::inverse(lookAtMatrix);

    rotationMatrix[3] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f); // Remove translation component

    glm::mat4 sphereModel = glm::translate(glm::mat4(1.0f), sphereWorldPos) *

                            rotationMatrix *

                            glm::scale(glm::mat4(1.0f), glm::vec3(sphereDiameter * panelLocalScale));

    // -- END OF MODIFICATION --

    float aberration = 0.05f + sphereHoverAlpha_ * 0.10f;
    
    glm::vec3 baseColor = isSelected ? selectedColor_ : inactiveColor_;
    glm::vec3 abColor1 = isSelected ? orange_aberration1_ : blue_aberration1_;
    glm::vec3 abColor2 = isSelected ? orange_aberration2_ : blue_aberration2_;

    renderer.RenderVRMenuWidget(view, projection, sphereModel, baseColor, aberration, alpha, abColor1, abColor2);

    if (textureId_ != 0) {
        // -- START OF MODIFICATION --

        const float ICON_FORWARD_FACTOR = 0.25f; // Small positive offset for concave effect

        const float DISPARITY_OFFSET = 0.04f;    // Horizontal stereo offset

        float scaledSphereDiameter = sphereDiameter * panelLocalScale;

        

        glm::vec3 z_axis = glm::vec3(rotationMatrix[2]);

        glm::vec3 iconWorldPos = sphereWorldPos + z_axis * (scaledSphereDiameter * ICON_FORWARD_FACTOR);

        int eyeIndex = renderer.getCurrentEyeIndex();

        // For CONCAVE (pushed in): left eye sees left (-), right eye sees right (+)

        float disparity = (eyeIndex == 0) ? -DISPARITY_OFFSET : DISPARITY_OFFSET;

        

        glm::vec3 cameraRight = glm::inverse(view)[0];

        iconWorldPos += cameraRight * (scaledSphereDiameter * disparity);

        

        glm::mat4 iconModel = glm::translate(glm::mat4(1.0f), iconWorldPos) *

                              rotationMatrix *

                              glm::scale(glm::mat4(1.0f), glm::vec3(scaledSphereDiameter * 0.8f));

        // -- END OF MODIFICATION --

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

